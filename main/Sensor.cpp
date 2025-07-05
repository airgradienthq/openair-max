/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "Sensor.h"

#include "AirgradientUART.h"
#include "AlphaSenseSensor.h"
#include "BQ25672.h"
#include "RemoteConfig.h"
#include "esp_log_level.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "MaxConfig.h"
#include <cmath>

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

Sensor::Sensor(i2c_master_bus_handle_t busHandle) : _busHandle(busHandle) {}

bool Sensor::init(RemoteConfig::Model model) {
  ESP_LOGI(TAG, "Initializing sensor...");
  esp_log_level_set(TAG, ESP_LOG_DEBUG);

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

  // BQ25672
  charger_ = new BQ25672;
  if (charger_->begin(_busHandle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed init charger");
    _chargerAvailable = false;
  } else {
    charger_->printSystemStatus();
    charger_->printControlAndConfiguration();
    charger_->update();

    float out;
    charger_->getBatteryPercentage(&out);
    ESP_LOGI(TAG, "Battery percentage %.1f%%", out);
    charger_->getChargingStatus();
  }

  if (model == RemoteConfig::O_M_1PPSTON_CE) {
    // Initialize alphasense sensor if model supported
    alphaSense_ = new AlphaSenseSensor();
    if (alphaSense_->initGas(_busHandle) == false) {
      _alphaSenseGasAvailable = false;
    }
    if (alphaSense_->initTemperature(_busHandle) == false) {
      _alphaSenseTempAvailable = false;
    }
  } else {
    ESP_LOGI(TAG, "Skip O3/NO2 sensor, not supported on this model");
    _alphaSenseGasAvailable = false;
    _alphaSenseTempAvailable = false;
  }

  // PMS 1
  agsPM1_ = new AirgradientIICSerial(_busHandle, SUBUART_CHANNEL_1, 0, 1);
  if (agsPM1_->begin(9600) == false) {
    ESP_LOGE(TAG, "Failed open serial for PM sensor 1");
    _pms1Available = false;
  } else {
    pms1_ = new PMS(agsPM1_);
    // NOTE: Since UART, need to check if its actually able to communicate?
  }

  // PMS 2
  agsPM2_ = new AirgradientIICSerial(_busHandle, SUBUART_CHANNEL_2, 0, 1);
  if (agsPM2_->begin(9600) == false) {
    ESP_LOGE(TAG, "Failed open serial for PM sensor 2");
    _pms2Available = false;
  } else {
    pms2_ = new PMS(agsPM2_);
  }

  // Warm up SGP41 and PMS
  _warmUpSensor();

  // Ensure PMS1 is available since the sensor using UART
  if (_pms1Available) {
    if (pms1_->isConnected() == false) {
      ESP_LOGE(TAG, "PMS1 is not connected");
      _pms1Available = false;
    }
  }
  if (_pms2Available) {
    if (pms2_->isConnected() == false) {
      ESP_LOGE(TAG, "PMS2 is not connected");
      _pms2Available = false;
    }
  }

  ESP_LOGI(TAG, "Initialize finish");

  return (_co2Available && _pms1Available && _pms2Available && _chargerAvailable &&
          _tvocNoxAvailable && _tempHumAvailable);
}

bool Sensor::startMeasures(int iterations, int intervalMs) {
  ESP_LOGI(TAG, "Start measures with %d iterations and interval in between %dms", iterations,
           intervalMs);

  // When starting, set all measures average values to invalid
  //  as indication no valid data from every iterations before saving to cache
  _averageMeasure.rco2 = DEFAULT_INVALID_CO2;
  _averageMeasure.atmp = DEFAULT_INVALID_TEMPERATURE;
  _averageMeasure.rhum = DEFAULT_INVALID_HUMIDITY;
  _averageMeasure.pm01 = DEFAULT_INVALID_PM;
  _averageMeasure.pm25 = DEFAULT_INVALID_PM;
  _averageMeasure.pm10 = DEFAULT_INVALID_PM;
  _averageMeasure.particleCount003 = DEFAULT_INVALID_PM;
  _averageMeasure.tvocRaw = DEFAULT_INVALID_TVOC;
  _averageMeasure.noxRaw = DEFAULT_INVALID_NOX;
  _averageMeasure.vBat = DEFAULT_INVALID_VOLT;
  _averageMeasure.vPanel = DEFAULT_INVALID_VOLT;
  _averageMeasure.o3WorkingElectrode = DEFAULT_INVALID_VOLT;
  _averageMeasure.o3AuxiliaryElectrode = DEFAULT_INVALID_VOLT;
  _averageMeasure.no2WorkingElectrode = DEFAULT_INVALID_VOLT;
  _averageMeasure.no2AuxiliaryElectrode = DEFAULT_INVALID_VOLT;
  _averageMeasure.afeTemp = DEFAULT_INVALID_VOLT;

  AirgradientClient::OpenAirMaxPayload iterationData;

  for (int i = 1; i <= iterations; i++) {
    uint32_t startIteration = MILLIS();

    if (_chargerAvailable) {
      // Call update schedule for BQ25672
      charger_->update();
    }

    // Attempt measure each sensor and sum each measures iteration
    _measure(iterationData);
    _applyIteration(iterationData);

    int timeSpendMs = MILLIS() - startIteration;
    int toDelay = intervalMs - timeSpendMs;
    if (toDelay < 0) {
      toDelay = 0;
    }
    ESP_LOGI(TAG, "Iteration %d takes %ums to finish, next iteration in %ums", i, timeSpendMs,
             toDelay);
    vTaskDelay(pdMS_TO_TICKS(toDelay));
  }

  // Now calculate the average based on total sum result of each measures iteration
  _calculateMeasuresAverage();
  // TODO: Validate the averaging always works!

  // TODO: _calculateMeasuresAverage should return if there's one or more measure data is invalid

  return true;
}

void Sensor::printMeasures() {
  ESP_LOGI(TAG, "<<< Average Measures >>>");
  ESP_LOGI(TAG, "CO2 : %d", _averageMeasure.rco2);
  ESP_LOGI(TAG, "Temperature : %.1f", _averageMeasure.atmp);
  ESP_LOGI(TAG, "Humidity : %.1f", _averageMeasure.rhum);
  ESP_LOGI(TAG, "PM1.0 : %.1f", _averageMeasure.pm01);
  ESP_LOGI(TAG, "PM2.5 : %.1f", _averageMeasure.pm25);
  ESP_LOGI(TAG, "PM10.0 : %.1f", _averageMeasure.pm10);
  ESP_LOGI(TAG, "PM 0.3 count : %d", _averageMeasure.particleCount003);
  ESP_LOGI(TAG, "TVOC Raw : %d", _averageMeasure.tvocRaw);
  ESP_LOGI(TAG, "NOx Raw : %d", _averageMeasure.noxRaw);
  ESP_LOGI(TAG, "VBAT : %.2f", _averageMeasure.vBat);
  ESP_LOGI(TAG, "VPanel : %.2f", _averageMeasure.vPanel);
  ESP_LOGI(TAG, "O3 WE: %.3fmV", _averageMeasure.o3WorkingElectrode);
  ESP_LOGI(TAG, "O3 AE: %.3fmV", _averageMeasure.o3AuxiliaryElectrode);
  ESP_LOGI(TAG, "NO2 WE: %.3fmV", _averageMeasure.no2WorkingElectrode);
  ESP_LOGI(TAG, "NO2 AE: %.3fmV", _averageMeasure.no2AuxiliaryElectrode);
  ESP_LOGI(TAG, "AFE Temperature: %.3fmV", _averageMeasure.afeTemp);
}

AirgradientClient::OpenAirMaxPayload Sensor::getLastAverageMeasure() { return _averageMeasure; }

bool Sensor::co2AttemptManualCalibration() {
  ESP_LOGI(TAG, "Attempt to do manual calibration");
  int error = co2_->startManualBackgroundCalibration();
  if (error != 0) {
    ESP_LOGE(TAG, "CO2 calibration Failed!");
    return false;
  }

  ESP_LOGD(TAG, "CO2 calibration Success!");
  return true;
}

void Sensor::_measure(AirgradientClient::OpenAirMaxPayload &data) {
  // Set measure data to invalid for indication if respective sensor failed
  data.rco2 = DEFAULT_INVALID_CO2;
  data.atmp = DEFAULT_INVALID_TEMPERATURE;
  data.rhum = DEFAULT_INVALID_HUMIDITY;
  data.pm01 = DEFAULT_INVALID_PM;
  data.pm25 = DEFAULT_INVALID_PM;
  data.pm10 = DEFAULT_INVALID_PM;
  data.particleCount003 = DEFAULT_INVALID_PM;
  data.tvocRaw = DEFAULT_INVALID_TVOC;
  data.noxRaw = DEFAULT_INVALID_NOX;
  data.vBat = DEFAULT_INVALID_VOLT;
  data.vPanel = DEFAULT_INVALID_VOLT;
  data.o3WorkingElectrode = DEFAULT_INVALID_VOLT;
  data.o3AuxiliaryElectrode = DEFAULT_INVALID_VOLT;
  data.no2WorkingElectrode = DEFAULT_INVALID_VOLT;
  data.no2AuxiliaryElectrode = DEFAULT_INVALID_VOLT;
  data.afeTemp = DEFAULT_INVALID_VOLT;

  if (_co2Available) {
    data.rco2 = co2_->read_sensor_measurements();
    ESP_LOGD(TAG, "CO2: %d", data.rco2);
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
    bool pms1ReadSuccess = false;
    PMS::Data pmData1;
    if (_pms1Available) {
      pms1_->clearBuffer();
      pms1_->requestRead();
      if (pms1_->readUntil(pmData1, 1000)) {
        ESP_LOGD(TAG, "{1} PM1.0 : %d", pmData1.pm_ae_1_0);
        ESP_LOGD(TAG, "{1} PM2.5 : %d", pmData1.pm_ae_2_5);
        ESP_LOGD(TAG, "{1} PM10.0 : %d", pmData1.pm_ae_10_0);
        ESP_LOGD(TAG, "{1} PM 0.3 count : %d", pmData1.pm_raw_0_3);
        pms1ReadSuccess = true;
      } else {
        ESP_LOGE(TAG, "{1} PMS no data");
      }
    }

    bool pms2ReadSuccess = false;
    PMS::Data pmData2;
    if (_pms2Available) {
      pms2_->clearBuffer();
      pms2_->requestRead();
      if (pms2_->readUntil(pmData2, 1000)) {
        ESP_LOGD(TAG, "{2} PM1.0 : %d", pmData2.pm_ae_1_0);
        ESP_LOGD(TAG, "{2} PM2.5 : %d", pmData2.pm_ae_2_5);
        ESP_LOGD(TAG, "{2} PM10.0 : %d", pmData2.pm_ae_10_0);
        ESP_LOGD(TAG, "{2} PM 0.3 count : %d", pmData2.pm_raw_0_3);
        pms2ReadSuccess = true;
      } else {
        ESP_LOGE(TAG, "{2} PMS no data");
      }
    }

    // Average if both success, if not, use only 1 or no data both if both failed
    if (pms1ReadSuccess && pms2ReadSuccess) {
      data.pm01 = (pmData1.pm_ae_1_0 + pmData2.pm_ae_1_0) / 2.0f;
      data.pm25 = (pmData1.pm_ae_2_5 + pmData2.pm_ae_2_5) / 2.0f;
      data.pm10 = (pmData1.pm_ae_10_0 + pmData2.pm_ae_10_0) / 2.0f;
      data.particleCount003 = (pmData1.pm_raw_0_3 + pmData2.pm_raw_0_3) / 2.0f;
    } else if (pms1ReadSuccess) {
      data.pm01 = pmData1.pm_ae_1_0;
      data.pm25 = pmData1.pm_ae_2_5;
      data.pm10 = pmData1.pm_ae_10_0;
      data.particleCount003 = pmData1.pm_raw_0_3;
    } else if (pms2ReadSuccess) {
      data.pm01 = pmData2.pm_ae_1_0;
      data.pm25 = pmData2.pm_ae_2_5;
      data.pm10 = pmData2.pm_ae_10_0;
      data.particleCount003 = pmData2.pm_raw_0_3;
    }
  }

  if (_chargerAvailable) {
    uint16_t output;
    esp_err_t err = charger_->getVBAT(&output);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Charger failed get VBAT");
    } else {
      data.vBat = output / 1000.0; // Convert from mV to V
      ESP_LOGD(TAG, "VBAT: %.2fV", data.vBat);
    }

    err = charger_->getVBUS(&output);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Charger failed get VBUS");
    } else {
      data.vPanel = output / 1000.0; // Convert from mV to V
      ESP_LOGD(TAG, "VBUS: %.2fV", data.vPanel);
    }
  }

  if (_alphaSenseGasAvailable) {
    data.o3WorkingElectrode = alphaSense_->getO3WorkingElectrode();
    data.o3AuxiliaryElectrode = alphaSense_->getO3AuxiliaryElectrode();
    data.no2WorkingElectrode = alphaSense_->getNO2WorkingElectrode();
    data.no2AuxiliaryElectrode = alphaSense_->getNO2AuxiliaryElectrode();
    ESP_LOGD(TAG, "O3 WE: %.3fmV", data.o3WorkingElectrode);
    ESP_LOGD(TAG, "O3 AE: %.3fmV", data.o3AuxiliaryElectrode);
    ESP_LOGD(TAG, "NO2 WE: %.3fmV", data.no2WorkingElectrode);
    ESP_LOGD(TAG, "NO2 AE: %.3fmV", data.no2AuxiliaryElectrode);
  }

  if (_alphaSenseTempAvailable) {
    data.afeTemp = alphaSense_->getTemperature();
    ESP_LOGD(TAG, "AFE Temperature: %.3fmV", data.afeTemp);
  }
}

void Sensor::_applyIteration(AirgradientClient::OpenAirMaxPayload &data) {
  if (IS_CO2_VALID(data.rco2)) {
    if (_averageMeasure.rco2 == DEFAULT_INVALID_CO2) {
      _averageMeasure.rco2 = data.rco2;
    } else {
      _averageMeasure.rco2 = _averageMeasure.rco2 + data.rco2;
    }
    _rco2IterationOkCount = _rco2IterationOkCount + 1;
  }

  if (IS_TEMPERATURE_VALID(data.atmp)) {
    if (_averageMeasure.atmp == DEFAULT_INVALID_TEMPERATURE) {
      _averageMeasure.atmp = data.atmp;
    } else {
      _averageMeasure.atmp = _averageMeasure.atmp + data.atmp;
    }
    _atmpIterationOkCount = _atmpIterationOkCount + 1;
  }

  if (IS_HUMIDITY_VALID(data.rhum)) {
    if (_averageMeasure.rhum == DEFAULT_INVALID_HUMIDITY) {
      _averageMeasure.rhum = data.rhum;
    } else {
      _averageMeasure.rhum = _averageMeasure.rhum + data.rhum;
    }
    _rhumIterationOkCount = _rhumIterationOkCount + 1;
  }

  if (IS_PM_VALID(data.pm01)) {
    if (_averageMeasure.pm01 == DEFAULT_INVALID_PM) {
      _averageMeasure.pm01 = data.pm01;
    } else {
      _averageMeasure.pm01 = _averageMeasure.pm01 + data.pm01;
    }
    _pm01IterationOkCount = _pm01IterationOkCount + 1;
  }

  if (IS_PM_VALID(data.pm25)) {
    if (_averageMeasure.pm25 == DEFAULT_INVALID_PM) {
      _averageMeasure.pm25 = data.pm25;
    } else {
      _averageMeasure.pm25 = _averageMeasure.pm25 + data.pm25;
    }
    _pm25IterationOkCount = _pm25IterationOkCount + 1;
  }

  if (IS_PM_VALID(data.pm10)) {
    if (_averageMeasure.pm10 == DEFAULT_INVALID_PM) {
      _averageMeasure.pm10 = data.pm10;
    } else {
      _averageMeasure.pm10 = _averageMeasure.pm10 + data.pm10;
    }
    _pm10IterationOkCount = _pm10IterationOkCount + 1;
  }

  if (IS_PM_VALID(data.particleCount003)) {
    if (_averageMeasure.particleCount003 == DEFAULT_INVALID_PM) {
      _averageMeasure.particleCount003 = data.particleCount003;
    } else {
      _averageMeasure.particleCount003 = _averageMeasure.particleCount003 + data.particleCount003;
    }
    _pm003CountIterationOkCount = _pm003CountIterationOkCount + 1;
  }

  if (IS_TVOC_VALID(data.tvocRaw)) {
    if (_averageMeasure.tvocRaw == DEFAULT_INVALID_TVOC) {
      _averageMeasure.tvocRaw = data.tvocRaw;
    } else {
      _averageMeasure.tvocRaw = _averageMeasure.tvocRaw + data.tvocRaw;
    }
    _tvocIterationOkCount = _tvocIterationOkCount + 1;
  }

  if (IS_NOX_VALID(data.noxRaw)) {
    if (_averageMeasure.noxRaw == DEFAULT_INVALID_NOX) {
      _averageMeasure.noxRaw = data.noxRaw;
    } else {
      _averageMeasure.noxRaw = _averageMeasure.noxRaw + data.noxRaw;
    }
    _noxIterationOkCount = _noxIterationOkCount + 1;
  }

  if (IS_VOLT_VALID(data.vBat)) {
    if (_averageMeasure.vBat == DEFAULT_INVALID_VOLT) {
      _averageMeasure.vBat = data.vBat;
    } else {
      _averageMeasure.vBat = _averageMeasure.vBat + data.vBat;
    }
    _vbatIterationOkCount = _vbatIterationOkCount + 1;
  }

  if (IS_VOLT_VALID(data.vPanel)) {
    if (_averageMeasure.vPanel == DEFAULT_INVALID_VOLT) {
      _averageMeasure.vPanel = data.vPanel;
    } else {
      _averageMeasure.vPanel = _averageMeasure.vPanel + data.vPanel;
    }
    _vpanelIterationOkCount = _vpanelIterationOkCount + 1;
  }

  if (IS_VOLT_VALID(data.o3WorkingElectrode)) {
    if (_averageMeasure.o3WorkingElectrode == DEFAULT_INVALID_VOLT) {
      _averageMeasure.o3WorkingElectrode = data.o3WorkingElectrode;
    } else {
      _averageMeasure.o3WorkingElectrode =
          _averageMeasure.o3WorkingElectrode + data.o3WorkingElectrode;
    }
    _o3WEIterationOkCount = _o3WEIterationOkCount + 1;
  }

  if (IS_VOLT_VALID(data.o3AuxiliaryElectrode)) {
    if (_averageMeasure.o3AuxiliaryElectrode == DEFAULT_INVALID_VOLT) {
      _averageMeasure.o3AuxiliaryElectrode = data.o3AuxiliaryElectrode;
    } else {
      _averageMeasure.o3AuxiliaryElectrode =
          _averageMeasure.o3AuxiliaryElectrode + data.o3AuxiliaryElectrode;
    }
    _o3AEIterationOkCount = _o3AEIterationOkCount + 1;
  }

  if (IS_VOLT_VALID(data.no2WorkingElectrode)) {
    if (_averageMeasure.no2WorkingElectrode == DEFAULT_INVALID_VOLT) {
      _averageMeasure.no2WorkingElectrode = data.no2WorkingElectrode;
    } else {
      _averageMeasure.no2WorkingElectrode =
          _averageMeasure.no2WorkingElectrode + data.no2WorkingElectrode;
    }
    _no2WEIterationOkCount = _no2WEIterationOkCount + 1;
  }

  if (IS_VOLT_VALID(data.no2AuxiliaryElectrode)) {
    if (_averageMeasure.no2AuxiliaryElectrode == DEFAULT_INVALID_VOLT) {
      _averageMeasure.no2AuxiliaryElectrode = data.no2AuxiliaryElectrode;
    } else {
      _averageMeasure.no2AuxiliaryElectrode =
          _averageMeasure.no2AuxiliaryElectrode + data.no2AuxiliaryElectrode;
    }
    _no2AEIterationOkCount = _no2AEIterationOkCount + 1;
  }

  if (IS_VOLT_VALID(data.afeTemp)) {
    if (_averageMeasure.afeTemp == DEFAULT_INVALID_VOLT) {
      _averageMeasure.afeTemp = data.afeTemp;
    } else {
      _averageMeasure.afeTemp = _averageMeasure.afeTemp + data.afeTemp;
    }
    _afeTempIterationOkCount = _afeTempIterationOkCount + 1;
  }
}

void Sensor::_warmUpSensor() {
  // Self test SGP41
  if (_tvocNoxAvailable) {
    sgp4x_self_test_result_t selfTestResult;
    esp_err_t result = sgp4x_execute_self_test(sgp_dev_hdl, &selfTestResult);
    if (result != ESP_OK) {
      ESP_LOGE(TAG, "sgp4x device self-test failed (%s)", esp_err_to_name(result));
    } else {
      ESP_LOGI(TAG, "VOC Pixel: %d", selfTestResult.pixels.voc_pixel_failed);
      ESP_LOGI(TAG, "NOX Pixel: %d", selfTestResult.pixels.nox_pixel_failed);
    }
  }

  // Warmup PM1 and PM2 while also do SGP conditioning
  // Only if sensor is available
  for (int i = 10; i >= 0; i--) {
    ESP_LOGI(TAG, "Warming up PMS and/or SGP41 sensors %d", i);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (_pms1Available) {
      pms1_->passiveMode();
    }
    if (_pms2Available) {
      pms2_->passiveMode();
    }
    if (_tvocNoxAvailable) {
      uint16_t sraw_voc;
      // NOTE: Use sgp4x_execute_compensated_conditioning() to pass rhum and atmp
      esp_err_t result = sgp4x_execute_conditioning(sgp_dev_hdl, &sraw_voc);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
      } else {
        ESP_LOGI(TAG, "SRAW VOC: %u", sraw_voc);
      }
    }
  }
}

void Sensor::_calculateMeasuresAverage() {
  if (_rco2IterationOkCount > 0) {
    _averageMeasure.rco2 = _averageMeasure.rco2 / _rco2IterationOkCount;
  }

  if (_atmpIterationOkCount > 0) {
    _averageMeasure.atmp = _averageMeasure.atmp / _atmpIterationOkCount;
  }

  if (_rhumIterationOkCount > 0) {
    _averageMeasure.rhum = _averageMeasure.rhum / _rhumIterationOkCount;
  }

  if (_pm01IterationOkCount > 0) {
    _averageMeasure.pm01 = _averageMeasure.pm01 / _pm01IterationOkCount;
  }

  if (_pm25IterationOkCount > 0) {
    _averageMeasure.pm25 = _averageMeasure.pm25 / _pm25IterationOkCount;
  }

  if (_pm10IterationOkCount > 0) {
    _averageMeasure.pm10 = _averageMeasure.pm10 / _pm10IterationOkCount;
  }

  if (_pm003CountIterationOkCount > 0) {
    _averageMeasure.particleCount003 =
        _averageMeasure.particleCount003 / _pm003CountIterationOkCount;
  }

  if (_tvocIterationOkCount > 0) {
    _averageMeasure.tvocRaw = _averageMeasure.tvocRaw / _tvocIterationOkCount;
  }

  if (_noxIterationOkCount > 0) {
    _averageMeasure.noxRaw = _averageMeasure.noxRaw / _noxIterationOkCount;
  }

  if (_vbatIterationOkCount > 0) {
    _averageMeasure.vBat = _averageMeasure.vBat / _vbatIterationOkCount;
  }

  if (_vpanelIterationOkCount > 0) {
    _averageMeasure.vPanel = _averageMeasure.vPanel / _vpanelIterationOkCount;
  }

  if (_o3WEIterationOkCount > 0) {
    _averageMeasure.o3WorkingElectrode = _averageMeasure.o3WorkingElectrode / _o3WEIterationOkCount;
  }

  if (_o3AEIterationOkCount > 0) {
    _averageMeasure.o3AuxiliaryElectrode =
        _averageMeasure.o3AuxiliaryElectrode / _o3AEIterationOkCount;
  }

  if (_no2WEIterationOkCount > 0) {
    _averageMeasure.no2WorkingElectrode =
        _averageMeasure.no2WorkingElectrode / _no2WEIterationOkCount;
  }

  if (_no2AEIterationOkCount > 0) {
    _averageMeasure.no2AuxiliaryElectrode =
        _averageMeasure.no2AuxiliaryElectrode / _no2AEIterationOkCount;
  }

  if (_afeTempIterationOkCount > 0) {
    _averageMeasure.afeTemp = _averageMeasure.afeTemp / _afeTempIterationOkCount;
  }
}
