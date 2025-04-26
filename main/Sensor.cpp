#include "Sensor.h"

#include "AirgradientUART.h"
#include "BQ25672.h"
#include "freertos/FreeRTOS.h"

#include "config.h"
#include "esp_log.h"

Sensor::Sensor(i2c_master_bus_handle_t busHandle) : _busHandle(busHandle) {}

bool Sensor::init() {
  // Sunlight sensor
  agsCO2_ = new AirgradientUART();
  if (!agsCO2_->begin(UART_PORT_SUNLIGHT, UART_BAUD_SUNLIGHT, UART_RX_SUNLIGHT, UART_TX_SUNLIGHT)) {
    ESP_LOGE(TAG, "Failed open serial for Sunlight");
    _co2Available = false;
  } else {
    co2_ = new Sunlight(*agsCO2_);
    co2_->read_sensor_id();
    // NOTE: Since UART, need to check if its actually able to communicate?
  }

  // SHT40
  // initialize i2c SHT configuration
  sht4x_config_t sht_cfg = I2C_SHT4X_CONFIG_DEFAULT;
  sht4x_init(_busHandle, &sht_cfg, &sht_dev_hdl);
  if (sht_dev_hdl == NULL) {
    _tempHumAvailable = false;
    ESP_LOGE(TAG, "Failed init temperature and humidity sensor");
  }

  // SGP41
  // initialize i2c SGP configuration
  sgp4x_config_t sgp_cfg = I2C_SGP41_CONFIG_DEFAULT;
  sgp4x_init(_busHandle, &sgp_cfg, &sgp_dev_hdl);
  if (sgp_dev_hdl == NULL) {
    _tvocNoxAvailable = false;
    ESP_LOGE(TAG, "Failed init TVOC & NOx sensor");
  }
  _warmUpSGP41();

  // BQ25672
  charger_ = new BQ25672;
  if (charger_->begin(_busHandle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed init charger");
    _chargerAvailable = false;
  } else {
    charger_->printSystemStatus();
    charger_->printControlAndConfiguration();
  }

  // PMS 1
  agsPM1_ = new AirgradientIICSerial(_busHandle, SUBUART_CHANNEL_1, 0, 1);
  if (agsPM1_->begin(9600) != 0) {
    ESP_LOGE(TAG, "Failed open serial for PM sensor 1");
    _pms1Available = false;
  } else {
    pms1_ = new PMS(agsPM1_);
    // NOTE: Since UART, need to check if its actually able to communicate?
  }

  // PMS 2
  agsPM2_ = new AirgradientIICSerial(_busHandle, SUBUART_CHANNEL_2, 0, 1);
  if (agsPM2_->begin(9600) != 0) {
    ESP_LOGE(TAG, "Failed open serial for PM sensor 2");
    _pms2Available = false;
  } else {
    pms2_ = new PMS(agsPM2_);
    // NOTE: Since UART, need to check if its actually able to communicate?
  }

  _warmUpPMS();

  return (_co2Available && _pms1Available && _pms2Available && _chargerAvailable &&
          _tvocNoxAvailable && _tempHumAvailable);
}

bool Sensor::startMeasure(int signalStrength, int iterations, int intervalMs) {
  ESP_LOGI(TAG, "Start measure with %d iterations and interval %d", iterations, intervalMs);
  AirgradientClient::OpenAirMaxPayload iterationData;
  _measure(iterationData);

  // TODO: Need to call charger update in iteration, if past schedule
  // TODO: Iteration measure
  // TODO: Printout what to cache basically the average

  return true;
}

void Sensor::_measure(AirgradientClient::OpenAirMaxPayload &data) {
  bool failedMeasureExist = false;

  // Set measure data to invalid for indication if read sensor failed
  data.rco2 = DEFAULT_INVALID_CO2;
  data.atmp = DEFAULT_INVALID_TEMPERATURE;
  data.rhum = DEFAULT_INVALID_HUMIDITY;
  data.pm01 = DEFAULT_INVALID_PM;
  data.pm25 = DEFAULT_INVALID_PM;
  data.pm10 = DEFAULT_INVALID_PM;
  data.tvocRaw = DEFAULT_INVALID_TVOC;
  data.noxRaw = DEFAULT_INVALID_NOX;
  data.vBatt = DEFAULT_INVALID_VBATT;
  data.vPanel = DEFAULT_INVALID_VPANEL;

  if (_co2Available) {
    ESP_LOGD(TAG, "CO2: %d", data.rco2);
    data.rco2 = co2_->read_sensor_measurements();
  }

  if (_tempHumAvailable) {
    float temperature, humidity;
    esp_err_t result = sht4x_get_measurement(sht_dev_hdl, &temperature, &humidity);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "sht4x device read failed (%s)", esp_err_to_name(result));
    } else {
      ESP_LOGD(TAG, "Temperature: %.2f °C", temperature);
      ESP_LOGD(TAG, "Relative humidity: %.2f %c", humidity, '%');
      data.atmp = temperature;
      data.rhum = humidity;
    }
  }

  if (_tvocNoxAvailable) {
    uint16_t tvocRaw;
    uint16_t noxRaw;
    // TODO: Check if temp and hum is valid first
    esp_err_t result =
        sgp4x_measure_compensated_signals(sgp_dev_hdl, data.atmp, data.rhum, &tvocRaw, &noxRaw);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
    } else {
      ESP_LOGD(TAG, "SRAW VOC: %u", tvocRaw);
      ESP_LOGD(TAG, "SRAW NOX: %u", noxRaw);
      data.tvocRaw = tvocRaw;
      data.noxRaw = noxRaw;
    }
  }

  if (_pms1Available || _pms2Available) {
    pms1_->requestRead();
    PMS::Data pmData1;
    if (pms1_->readUntil(pmData1, 3000)) {
      ESP_LOGD(TAG, "{1} PM1.0 : %d", pmData1.pm_ae_1_0);
      ESP_LOGD(TAG, "{1} PM2.5 : %d", pmData1.pm_ae_2_5);
      ESP_LOGD(TAG, "{1} PM10.0 : %d", pmData1.pm_ae_10_0);
      ESP_LOGD(TAG, "{1} PM 0.3 count : %d", pmData1.pm_raw_0_3);
    } else {
      ESP_LOGE(TAG, "{1} PMS no data");
    }

    pms2_->requestRead();
    PMS::Data pmData2;
    if (pms2_->readUntil(pmData2, 3000)) {
      ESP_LOGD(TAG, "{2} PM1.0 : %d", pmData2.pm_ae_1_0);
      ESP_LOGD(TAG, "{2} PM2.5 : %d", pmData2.pm_ae_2_5);
      ESP_LOGD(TAG, "{2} PM10.0 : %d", pmData2.pm_ae_10_0);
      ESP_LOGD(TAG, "{2} PM 0.3 count : %d", pmData2.pm_raw_0_3);
    } else {
      ESP_LOGE(TAG, "{2} PMS no data");
    }

    // TODO: Average the data first for PM, check each value, use 1 sensor if only 1 available, ignore if both not avail
  }

  if (_chargerAvailable) {
    uint16_t output;
    esp_err_t err = charger_->getVBAT(&output);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Charger failed get VBAT");
    } else {
      ESP_LOGD(TAG, "VBAT: %dmv", output);
      data.vBatt = output;
    }

    err = charger_->getVBUS(&output);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Charger failed get VBUS");
    } else {
      ESP_LOGD(TAG, "VBUS: %dmv", output);
      data.vPanel = output;
    }
  }
}

void Sensor::_warmUpSGP41() {
  // Self test
  sgp4x_self_test_result_t selfTestResult;
  esp_err_t result = sgp4x_execute_self_test(sgp_dev_hdl, &selfTestResult);
  if (result != ESP_OK) {
    ESP_LOGE(TAG, "sgp4x device self-test failed (%s)", esp_err_to_name(result));
  } else {
    ESP_LOGI(TAG, "VOC Pixel: %d", selfTestResult.pixels.voc_pixel_failed);
    ESP_LOGI(TAG, "NOX Pixel: %d", selfTestResult.pixels.nox_pixel_failed);
  }

  // TVOC Conditioning
  for (int i = 10; i >= 0; i--) {
    uint16_t sraw_voc;
    ESP_LOGI(TAG, "Warming up TVOC and NOx sensor %d", i);
    // NOTE: Use sgp4x_execute_compensated_conditioning() to pass rhum and atmp
    esp_err_t result = sgp4x_execute_conditioning(sgp_dev_hdl, &sraw_voc);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
    } else {
      ESP_LOGI(TAG, "SRAW VOC: %u", sraw_voc);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second * 10 iterations = 10-seconds
  }
}

void Sensor::_warmUpPMS() {
  // Warmup PM1 and PM2
  for (int i = 10; i >= 0; i--) {
    ESP_LOGI(TAG, "Warming up PM sensors %d", i);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (_pms1Available) {
      pms1_->passiveMode();
    }
    if (_pms2Available) {
      pms2_->passiveMode();
    }
  }
}
