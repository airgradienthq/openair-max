#include <cstdint>
#include <stdio.h>
#include <inttypes.h>
#include <string>
#include "AirgradientSerial.h"
#include "esp_log_level.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "AirgradientUART.h"
#include "AirgradientIICSerial.h"
#include "driver/gpio.h"
#include "Sunlight.h"
#include "PMS.h"
#include "sht4x.h"
#include "sgp4x.h"
#include "sensirion_gas_index_algorithm.h"
#include "BQ25672.h"
#include "StatusLed.h"

static const gpio_num_t EN_CO2 = GPIO_NUM_15;
static const gpio_num_t EN_PM1 = GPIO_NUM_3;
static const gpio_num_t EN_PM2 = GPIO_NUM_11;
static const gpio_num_t IO_WDT = GPIO_NUM_2;
static const gpio_num_t IO_CE_CARD = GPIO_NUM_22;

static const char *const TAG = "APP";
static const uint8_t CO2_SUNLIGHT_ADDR = 0x68;

#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT 0

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

void reset() {
  gpio_set_level(IO_WDT, 1);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(IO_WDT, 0);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Hello world");

  StatusLed statusLed(GPIO_NUM_10);
  statusLed.start();
  statusLed.set(StatusLed::On);

  // Enable Both PM
  gpio_reset_pin(EN_PM1);
  gpio_set_direction(EN_PM1, GPIO_MODE_OUTPUT);
  gpio_set_level(EN_PM1, 1);
  gpio_reset_pin(EN_PM2);
  gpio_set_direction(EN_PM2, GPIO_MODE_OUTPUT);
  gpio_set_level(EN_PM2, 1);

  // Enable Sunlight
  gpio_reset_pin(EN_CO2);
  gpio_set_direction(EN_CO2, GPIO_MODE_OUTPUT);
  gpio_set_level(EN_CO2, 1);

  // watchdog
  gpio_reset_pin(IO_WDT);
  gpio_set_direction(IO_WDT, GPIO_MODE_OUTPUT);
  gpio_set_level(IO_WDT, 0);

  // Cellular card
  gpio_reset_pin(IO_CE_CARD);
  gpio_set_direction(IO_CE_CARD, GPIO_MODE_OUTPUT);
  gpio_set_level(IO_CE_CARD, 1);

  vTaskDelay(pdMS_TO_TICKS(100));

  // Sunlight sensor
  AirgradientSerial *agsCO2 = new AirgradientUART;
  if (!agsCO2->open(0, 9600, 0, 1)) {
    ESP_LOGE(TAG, "Failed open serial for Sunlight");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  Sunlight co2(*agsCO2);
  vTaskDelay(pdMS_TO_TICKS(100));
  co2.read_sensor_id(CO2_SUNLIGHT_ADDR);

  // Configure I2C master bus
  i2c_master_bus_config_t bus_cfg = {
      .i2c_port = I2C_MASTER_PORT,
      .sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO,
      .scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      // .flags.enable_internal_pullup = true,
  };
  bus_cfg.flags.enable_internal_pullup = true;
  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));
  vTaskDelay(pdMS_TO_TICKS(2000));

  // initialize i2c SHT configuration
  sht4x_config_t sht_cfg = I2C_SHT4X_CONFIG_DEFAULT;
  sht4x_handle_t sht_dev_hdl;
  // SHT40
  sht4x_init(bus_handle, &sht_cfg, &sht_dev_hdl);
  if (sht_dev_hdl == NULL) {
    ESP_LOGE(TAG, "sht4x handle init failed");
    assert(sht_dev_hdl);
  }

  // initialize i2c SGP configuration
  sgp4x_config_t sgp_cfg = I2C_SGP41_CONFIG_DEFAULT;
  sgp4x_handle_t sgp_dev_hdl;
  bool dev_self_tested = false;
  bool dev_conditioned = false;

  // initialize gas index parameters
  GasIndexAlgorithmParams voc_params;
  GasIndexAlgorithm_init(&voc_params, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);
  GasIndexAlgorithmParams nox_params;
  GasIndexAlgorithm_init(&nox_params, GasIndexAlgorithm_ALGORITHM_TYPE_NOX);
  int32_t indexOffset;
  int32_t learningTimeOffsetHours;
  int32_t learningTimeGainHours;
  int32_t gatingMaxDurationMin;
  int32_t stdInitial;
  int32_t gainFactor;

  // Tune VOC params
  GasIndexAlgorithm_get_tuning_parameters(&voc_params, &indexOffset, &learningTimeOffsetHours,
                                          &learningTimeGainHours, &gatingMaxDurationMin,
                                          &stdInitial, &gainFactor);
  learningTimeOffsetHours = 12;
  GasIndexAlgorithm_set_tuning_parameters(&voc_params, indexOffset, learningTimeOffsetHours,
                                          learningTimeGainHours, gatingMaxDurationMin, stdInitial,
                                          gainFactor);
  // Tune NOx params
  GasIndexAlgorithm_get_tuning_parameters(&nox_params, &indexOffset, &learningTimeOffsetHours,
                                          &learningTimeGainHours, &gatingMaxDurationMin,
                                          &stdInitial, &gainFactor);
  learningTimeOffsetHours = 12;
  GasIndexAlgorithm_set_tuning_parameters(&nox_params, indexOffset, learningTimeOffsetHours,
                                          learningTimeGainHours, gatingMaxDurationMin, stdInitial,
                                          gainFactor);
  // Initialize SGP41
  sgp4x_init(bus_handle, &sgp_cfg, &sgp_dev_hdl);
  if (sgp_dev_hdl == NULL) {
    ESP_LOGE(TAG, "sgp4x handle init failed");
    assert(sgp_dev_hdl);
  }

  // BQ25672
  BQ25672 charger;
  if (charger.begin(bus_handle) != ESP_OK) {
    ESP_LOGE(TAG, "Failed init charger");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  charger.printSystemStatus();
  charger.printControlAndConfiguration();

  // PMS 1
  AirgradientSerial *agsPM1 = new AirgradientIICSerial(bus_handle, SUBUART_CHANNEL_1, 0, 1);
  if (agsPM1->begin(9600) != 0) {
    ESP_LOGE(TAG, "Failed open serial for PMS2");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  PMS pms1(agsPM1);

  // PMS 2
  AirgradientSerial *agsPM2 = new AirgradientIICSerial(bus_handle, SUBUART_CHANNEL_2, 0, 1);
  if (agsPM2->begin(9600) != 0) {
    ESP_LOGE(TAG, "Failed open serial for PMS2");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  PMS pms2(agsPM2);

  statusLed.set(StatusLed::Blink, 5000, 1000);

  // vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Forever loop");
  bool warmup = true;
  if (warmup) {
    for (int i = 10; i >= 0; i--) {
      ESP_LOGI(TAG, "Warming up PMS sensors %d", i);
      vTaskDelay(pdMS_TO_TICKS(1000));
      pms1.passiveMode();
      pms2.passiveMode();
    }
  }

  uint32_t readCycleTime = MILLIS();

  float temperature, humidity;

  while (1) {
    if ((MILLIS() - readCycleTime) > 10000) {
      statusLed.set(StatusLed::Blink, 0, 500);

      // SUNLIGHT
      auto co2Value = co2.read_sensor_measurements(CO2_SUNLIGHT_ADDR);
      ESP_LOGI(TAG, "CO2: %d", co2Value);

      // SHT40
      esp_err_t result = sht4x_get_measurement(sht_dev_hdl, &temperature, &humidity);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "sht4x device read failed (%s)", esp_err_to_name(result));
      } else {
        ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
        ESP_LOGI(TAG, "Relative humidity: %.2f %c", humidity, '%');
      }

      // PMS
      pms1.requestRead();
      PMS::Data data;
      if (pms1.readUntil(data, 3000)) {
        ESP_LOGI(TAG, "{1} PM1.0 = %d", data.pm_ae_1_0);
        ESP_LOGI(TAG, "{1} PM2.5 = %d", data.pm_ae_2_5);
        ESP_LOGI(TAG, "{1} PM10.0 = %d", data.pm_ae_10_0);
        ESP_LOGI(TAG, "{1} PM 0.3 count = %d", data.pm_raw_0_3);
      } else {
        ESP_LOGW(TAG, "{1} No data");
      }

      pms2.requestRead();
      if (pms2.readUntil(data, 3000)) {
        ESP_LOGI(TAG, "{2} PM1.0 = %d", data.pm_ae_1_0);
        ESP_LOGI(TAG, "{2} PM2.5 = %d", data.pm_ae_2_5);
        ESP_LOGI(TAG, "{2} PM10.0 = %d", data.pm_ae_10_0);
        ESP_LOGI(TAG, "{2} PM 0.3 count = %d", data.pm_raw_0_3);
      } else {
        ESP_LOGW(TAG, "{2} No data");
      }

      // Serial.println("=== ADC Measurement Registers ===");
      uint16_t resultRaw, result_uint;
      int16_t result_int;
      float result_float;
      if (charger.getVBUSRaw(&resultRaw) == ESP_OK && charger.getVBUS(&result_uint) == ESP_OK) {
        ESP_LOGI(TAG, "VBUS Raw (Hex): 0x%04X | VBUS: %d mV", resultRaw, result_uint);
      }

      // ESP_LOGI(TAG, "IBUS RaW (Hex): 0x%04C | IBUS: %d mA\n", charger.readRegister(0x31, 2),
      //          charger.readRegister(0x31, 2));

      if (charger.getVBATRaw(&resultRaw) == ESP_OK && charger.getVBAT(&result_uint) == ESP_OK) {
        ESP_LOGI(TAG, "VBAT Raw (Hex): 0x%04X | VBAT: %d mV", resultRaw, result_uint);
      }
      if (charger.getIBATRaw(&resultRaw) == ESP_OK &&
          charger.getBatteryCurrent(&result_int) == ESP_OK) {
        ESP_LOGI(TAG, "IBAT Raw (Hex): 0x%04X | IBAT: %d mA", resultRaw, result_int);
      }
      if (charger.getVSYSRaw(&resultRaw) == ESP_OK && charger.getVSYS(&result_uint) == ESP_OK) {
        ESP_LOGI(TAG, "VSYS Raw (Hex): 0x%04X | VSYS: %d mV", resultRaw, result_uint);
      }
      if (charger.getBatteryPercentage(&result_float) == ESP_OK) {
        ESP_LOGI(TAG, "Battery %%: %.2f %%", result_float);
      }
      if (charger.getTemperatureRaw(&resultRaw) == ESP_OK &&
          charger.getTemperature(&result_float) == ESP_OK) {
        ESP_LOGI(TAG, "Temp Raw (Hex): 0x%04X | Temp: %.2f C", resultRaw, result_float);
      }

      charger.getChargingStatus();

      // Feed watchdog
      reset();

      readCycleTime = MILLIS();

      statusLed.set(StatusLed::Off);
    }

    charger.update();

    // SGP41
    if (dev_self_tested == false) {
      sgp4x_self_test_result_t self_test_result;
      esp_err_t result = sgp4x_execute_self_test(sgp_dev_hdl, &self_test_result);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "sgp4x device self-test failed (%s)", esp_err_to_name(result));
      } else {
        ESP_LOGI(TAG, "VOC Pixel:   %d", self_test_result.pixels.voc_pixel_failed);
        ESP_LOGI(TAG, "NOX Pixel:   %d", self_test_result.pixels.nox_pixel_failed);
      }
      dev_self_tested = true;
    }
    if (dev_conditioned == false) {
      statusLed.set(StatusLed::Blink, 0, 250);
      for (int i = 0; i < 10; i++) {
        uint16_t sraw_voc;
        // NOTE: Use sgp4x_execute_compensated_conditioning() to pass rhum and atmp
        esp_err_t result = sgp4x_execute_conditioning(sgp_dev_hdl, &sraw_voc);
        if (result != ESP_OK) {
          ESP_LOGE(TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
        } else {
          ESP_LOGI(TAG, "SRAW VOC: %u", sraw_voc);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second * 10 iterations = 10-seconds
      }
      statusLed.set(StatusLed::Off);
      dev_conditioned = true;
    } else {
      uint16_t sraw_voc;
      uint16_t sraw_nox;
      int32_t voc_index;
      int32_t nox_index;
      esp_err_t result = sgp4x_measure_compensated_signals(sgp_dev_hdl, temperature, humidity,
                                                           &sraw_voc, &sraw_nox);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "sgp4x device conditioning failed (%s)", esp_err_to_name(result));
      } else {
        GasIndexAlgorithm_process(&voc_params, sraw_voc, &voc_index);
        GasIndexAlgorithm_process(&nox_params, sraw_nox, &nox_index);

        ESP_LOGI(TAG, "SRAW VOC: %u | VOC Index: %d", sraw_voc, (int)voc_index);
        ESP_LOGI(TAG, "SRAW NOX: %u | NOX Index: %d", sraw_nox, (int)nox_index);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
