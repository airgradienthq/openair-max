#include <cstdint>
#include <stdio.h>
#include <inttypes.h>
#include <string>

#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

// #include "AirgradientSerial.h"
// #include "AirgradientUART.h"
// #include "AirgradientIICSerial.h"
// #include "airgradientClient.h"
// #include "airgradientCellularClient.h"
// #include "cellularModuleA7672xx.h"

#include "config.h"
#include "PayloadCache.h"
#include "StatusLed.h"
#include "Sensor.h"

RTC_DATA_ATTR unsigned long wakeUpCount = 0;

// Global Vars
static const char *const TAG = "APP";
static std::string g_serialNumber;

// Prototype functions
static void enableIO(bool enableCECard);
static void resetExtWatchdog();
static void printWakeupReason();
static void goSleep();
static std::string buildSerialNumber();

extern "C" void app_main(void) {
  // Give time for logs to initialized (solving wake up cycle no logs)
  esp_log_level_set("*", ESP_LOG_INFO);
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "MAX!");

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // TODO: Print firmware version

  g_serialNumber = buildSerialNumber();
  ESP_LOGI(TAG, "Serial number: %s", g_serialNumber.c_str());

  printWakeupReason();

  StatusLed statusLed(IO_LED_INDICATOR);
  statusLed.start();
  statusLed.set(StatusLed::On);

  // Initialize and enable all IO required
  enableIO(true);

  // Reset external WDT
  resetExtWatchdog();

  ESP_LOGI(TAG, "Wait for sensors to warmup before initialization");
  vTaskDelay(pdMS_TO_TICKS(2000));

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

  Sensor sensor(bus_handle);
  if (!sensor.init()) {
    ESP_LOGW(
        TAG,
        "One or more sensor were failed to initialize, will not measure those on this iteration");
  }

  PayloadCache payloadCache(MAX_PAYLOAD_CACHE);

  statusLed.set(StatusLed::Blink, 0, 1000);
  if (sensor.startMeasures(20, 2000)) {
    sensor.printMeasures();
    auto averageMeasures = sensor.getLastAverageMeasure();
    payloadCache.push(&averageMeasures);
  }

  // Its finish measure, prepare to sleep
  statusLed.set(StatusLed::Blink, 0, 250);

  std::vector<AirgradientClient::OpenAirMaxPayload> payloads;
  PayloadType tmp;
  ESP_LOGI(TAG, "cache size: %d", payloadCache.getSize());
  if (payloadCache.getSize() >= 3) {
    for (int i = 0; i < 3; i++) {
      payloadCache.peekAtIndex(i, &tmp);
      payloads.push_back(tmp);
    }
    payloadCache.clean();
  }

  for (auto &v : payloads) {
    ESP_LOGI(TAG, "----------------------------");
    ESP_LOGI(TAG, "CO2 : %d", v.rco2);
    ESP_LOGI(TAG, "Temperature : %.1f", v.atmp);
    ESP_LOGI(TAG, "Humidity : %.1f", v.rhum);
    ESP_LOGI(TAG, "PM1.0 : %.1f", v.pm01);
    ESP_LOGI(TAG, "PM2.5 : %.1f", v.pm25);
    ESP_LOGI(TAG, "PM10.0 : %.1f", v.pm10);
    ESP_LOGI(TAG, "PM 0.3 count : %d", v.particleCount003);
    ESP_LOGI(TAG, "TVOC Raw : %d", v.tvocRaw);
    ESP_LOGI(TAG, "NOx Raw : %d", v.noxRaw);
    ESP_LOGI(TAG, "VBAT : %.2f", v.vBat);
    ESP_LOGI(TAG, "VPanel : %.2f", v.vPanel);
  }

  resetExtWatchdog();

  int toSleepMs = 1 * 60000;
  ESP_LOGI(TAG, "Sleeping");
  esp_sleep_enable_timer_wakeup(toSleepMs * 1000);
  statusLed.set(StatusLed::Off);
  vTaskDelay(pdMS_TO_TICKS(1000));
  esp_deep_sleep_start();

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void resetExtWatchdog() {
  ESP_LOGI(TAG, "Watchdog reset");
  gpio_set_level(IO_WDT, 1);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(IO_WDT, 0);
}

void enableIO(bool enableCECard) {
  // watchdog
  gpio_reset_pin(IO_WDT);
  gpio_set_direction(IO_WDT, GPIO_MODE_OUTPUT);
  gpio_set_level(IO_WDT, 0);

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

  // Cellular card
  if (enableCECard) {
    gpio_reset_pin(EN_CE_CARD);
    gpio_set_direction(EN_CE_CARD, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_CE_CARD, 1);
  }
}

void printWakeupReason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
  case ESP_SLEEP_WAKEUP_EXT0:
    ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_IO");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    ESP_LOGI(TAG, "Wakeup caused by external signal using RTC_CNTL");
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    ESP_LOGI(TAG, "Wakeup caused by timer");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    ESP_LOGI(TAG, "Wakeup caused by touchpad");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    ESP_LOGI(TAG, "Wakeup caused by ULP program");
    break;
  default:
    ESP_LOGI(TAG, "Wakeup was not caused by deep sleep: %d", wakeup_reason);
    ESP_LOGI(TAG, "Wakeup count: %lu", wakeUpCount);
    return;
  }

  ++wakeUpCount;
  ESP_LOGI(TAG, "Wakeup count: %lu", wakeUpCount);
}

void goSleep() {
  resetExtWatchdog();
  // TODO
}

std::string buildSerialNumber() {
  // Initialize Wi-Fi driver
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  uint8_t mac_address[6];
  esp_err_t err = esp_wifi_get_mac(WIFI_IF_STA, mac_address); // Get MAC address of Wi-Fi interface
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed build serial number, get wifi mac addr failed (%s)",
             esp_err_to_name(err));
    return {};
  }

  char result[13] = {0};
  snprintf(result, sizeof(result), "%02x%02x%02x%02x%02x%02x", mac_address[0], mac_address[1],
           mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
  std::string sn = std::string(result);

  // Deinitialize Wi-Fi after use
  esp_wifi_deinit();

  return sn;
}
