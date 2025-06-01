/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include <cstdint>
#include <stdio.h>
#include <inttypes.h>
#include <string>

#include <fcntl.h>
#include "esp_console.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"

#include "MaxConfig.h"
#include "RemoteConfig.h"
#include "PayloadCache.h"
#include "StatusLed.h"
#include "Sensor.h"
#include "AirgradientSerial.h"
#include "AirgradientUART.h"
#include "airgradientClient.h"
#include "cellularModule.h"
#include "airgradientCellularClient.h"
#include "cellularModuleA7672xx.h"
#include "airgradientOtaCellular.h"

#define CONSOLE_MAX_CMDLINE_ARGS 8
#define CONSOLE_MAX_CMDLINE_LENGTH 256
#define CONSOLE_PROMPT_MAX_LEN (32)

// Wake up counter that saved on Low Power memory
RTC_DATA_ATTR unsigned long xWakeUpCounter = 0;

// Global Vars
static const char *const TAG = "APP";
static std::string g_serialNumber;
static bool g_networkReady = false;
static std::string g_fimwareVersion;
static RemoteConfig g_remoteConfig;
static StatusLed g_statusLed(IO_LED_INDICATOR);
static AirgradientSerial *g_ceAgSerial = nullptr;
static CellularModule *g_cellularCard = nullptr;
static AirgradientClient *g_agClient = nullptr;

/**
 * Re-initialize console for logging
 */
static void initConsole();

void initGPIO();

/**
 * Reset monitor external watchdog timer
 *   with assumption GPIO already initialized before calling this function
 */
static void resetExtWatchdog();

/**
 * Helper to print out the reason system wake up from deepsleep
 */
static void printWakeupReason(esp_sleep_wakeup_cause_t reason);

/**
 * Build monitor serial number from WiFi mac address
 */
static std::string buildSerialNumber();

/**
 * Attempt to initialize and connect to cellular network
 * If failed once, it will not re-attempt to initialize for its wake up cycle
 * Called when post measure, check remote configuraiton and check for firmware update
 */
static bool initializeCellularNetwork(unsigned long wakeUpCounter);

/**
 * Retrieve currently running firmware version
 */
static std::string getFirmwareVersion();

// Return false if failed init network or failed send
// TODO: Add description
static bool checkRemoteConfiguration(unsigned long wakeUpCounter);
static bool sendMeasuresWhenReady(unsigned long wakeUpCounter, PayloadCache &payloadCache);
static bool checkForFirmwareUpdate(unsigned long wakeUpCounter);

extern "C" void app_main(void) {
  // Re-initialize console for logging only if it just wake up after deepsleep
  esp_sleep_wakeup_cause_t wakeUpReason = esp_sleep_get_wakeup_cause();
  if (wakeUpReason != ESP_SLEEP_WAKEUP_UNDEFINED) {
    initConsole();
    ++xWakeUpCounter;
    ESP_LOGI(TAG, "Wakeup count: %lu", xWakeUpCounter);
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
  printWakeupReason(wakeUpReason);

  // Initialize every peripheral GPIOs to OFF state
  initGPIO();

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  vTaskDelay(pdMS_TO_TICKS(100));

  uint32_t wakeUpMillis = MILLIS() - 1000; // minus with previous delay
  ESP_LOGI(TAG, "MAX!");

  g_statusLed.start();
  if (xWakeUpCounter == 0) {
    // Only turn on led indicator when it just powered on, not wakeup from sleep
    g_statusLed.set(StatusLed::On);
  }

  g_fimwareVersion = getFirmwareVersion();
  ESP_LOGI(TAG, "Firmware version: %s", g_fimwareVersion.c_str());

  g_serialNumber = buildSerialNumber();
  ESP_LOGI(TAG, "Serial number: %s", g_serialNumber.c_str());

  // Load remote configuration that saved on NVS
  g_remoteConfig.load();

  // Run led test if requested
  if (g_remoteConfig.isLedTestRequested()) {
    g_statusLed.set(StatusLed::Blink, 5000, 100);
    vTaskDelay(pdMS_TO_TICKS(5000));
    g_remoteConfig.resetLedTestRequested();
  }

  // Reset external WDT
  resetExtWatchdog();

  ESP_LOGI(TAG, "Wait for sensors to warmup before initialization");
  vTaskDelay(pdMS_TO_TICKS(2000));
  g_statusLed.set(StatusLed::Off);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Turn ON PMS and CO2 sensor load switch
  gpio_set_level(EN_PMS, 1);
  vTaskDelay(pdMS_TO_TICKS(2000));
  gpio_set_level(EN_CO2, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

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
    g_statusLed.set(StatusLed::Blink, 500, 100);
    ESP_LOGW(
        TAG,
        "One or more sensor were failed to initialize, will not measure those on this iteration");
  }

  if (g_remoteConfig.isCo2CalibrationRequested()) {
    // TODO: Implement!
  }

  // Start measure sensor sequence that if success,
  //   push new measure cycle to payload cache to send later
  PayloadCache payloadCache(MAX_PAYLOAD_CACHE);
  if (sensor.startMeasures(DEFAULT_MEASURE_ITERATION_COUNT,
                           DEFAULT_MEASURE_INTERVAL_MS_PER_ITERATION)) {
    sensor.printMeasures();
    auto averageMeasures = sensor.getLastAverageMeasure();
    payloadCache.push(&averageMeasures);
  }

  // Turn OFF PMS and CO2 sensor load switch
  gpio_set_level(EN_PMS, 0);
  gpio_set_level(EN_CO2, 0);
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Optimization: copy from LP memory so will not always call from LP memory
  int wakeUpCounter = xWakeUpCounter;

  sendMeasuresWhenReady(wakeUpCounter, payloadCache);
  checkRemoteConfiguration(wakeUpCounter);
  checkForFirmwareUpdate(wakeUpCounter);

  // Only poweroff when all transmission attempt is done
  if (g_ceAgSerial != nullptr || g_networkReady) {
    g_cellularCard->powerOff();
  }
  // Turn OFF Cellular Card load switch
  gpio_set_level(EN_CE_CARD, 0);

  // Reset external watchdog before sleep to make sure its not trigger while in sleep
  //   before system wakeup
  resetExtWatchdog();

  // TODO: Print cache size before sleep

  // Calculate how long to sleep to keep measurement cycle the same
  uint32_t aliveTimeSpendMillis = MILLIS() - wakeUpMillis;
  int toSleepMs = (g_remoteConfig.getConfigSchedule().pm02 * 1000) - aliveTimeSpendMillis;
  if (toSleepMs < 0) {
    // TODO: if its 0 means, no need to sleep, right? if so need to move to loop
    toSleepMs = 0;
  }
  ESP_LOGI(TAG, "Will sleep for %dms", toSleepMs);
  esp_sleep_enable_timer_wakeup(toSleepMs * 1000);
  vTaskDelay(pdMS_TO_TICKS(1000));
  g_statusLed.disable();
  esp_deep_sleep_start();

  // Will never go here

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void initConsole() {
  fflush(stdout);
  fsync(fileno(stdout));
  esp_console_deinit();

  /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  /* Move the caret to the beginning of the next line on '\n' */
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

  /* Enable blocking mode on stdin and stdout */
  fcntl(fileno(stdout), F_SETFL, 0);
  fcntl(fileno(stdin), F_SETFL, 0);

  usb_serial_jtag_driver_config_t jtag_config = {
      .tx_buffer_size = 256,
      .rx_buffer_size = 256,
  };

  /* Install USB-SERIAL-JTAG driver for interrupt-driven reads and writes */
  ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&jtag_config));

  /* Tell vfs to use usb-serial-jtag driver */
  usb_serial_jtag_vfs_use_driver();

  /* Initialize the console */
  esp_console_config_t console_config = {.max_cmdline_length = CONSOLE_MAX_CMDLINE_LENGTH,
                                         .max_cmdline_args = CONSOLE_MAX_CMDLINE_ARGS,
#if CONFIG_LOG_COLORS
                                         .hint_color = atoi(LOG_COLOR_CYAN)
#endif
  };
  ESP_ERROR_CHECK(esp_console_init(&console_config));
}

void resetExtWatchdog() {
  ESP_LOGI(TAG, "Watchdog reset");
  gpio_set_level(IO_WDT, 1);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(IO_WDT, 0);
}

void initGPIO() {
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << IO_WDT) | (1ULL << EN_PMS) | (1ULL << EN_CO2) |
                                           (1ULL << EN_CE_CARD),
                           .mode = GPIO_MODE_OUTPUT,
                           .pull_up_en = GPIO_PULLUP_DISABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  // TODO: Add reason why, and why watchdog not needed
  gpio_set_drive_capability(IO_WDT, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_PMS, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_CO2, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability(EN_CE_CARD, GPIO_DRIVE_CAP_3);

  gpio_set_level(IO_WDT, 0);
  gpio_set_level(EN_PMS, 0);
  gpio_set_level(EN_CO2, 0);
  gpio_set_level(EN_CE_CARD, 0);
}

void printWakeupReason(esp_sleep_wakeup_cause_t reason) {
  switch (reason) {
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
    ESP_LOGI(TAG, "Wakeup was not caused by deep sleep: %d", reason);
    ESP_LOGI(TAG, "Wakeup counter: %lu", xWakeUpCounter);
  }
}

std::string buildSerialNumber() {
  uint8_t mac_address[6];
  esp_err_t err = esp_read_mac(mac_address, ESP_MAC_WIFI_STA);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get MAC address (%s)", esp_err_to_name(err));
    return {};
  }

  char result[13] = {0};
  snprintf(result, sizeof(result), "%02x%02x%02x%02x%02x%02x", mac_address[0], mac_address[1],
           mac_address[2], mac_address[3], mac_address[4], mac_address[5]);
  std::string sn = std::string(result);

  return sn;
}

std::string getFirmwareVersion() {
  const esp_app_desc_t *app_desc = esp_app_get_description();
  return app_desc->version;
}

bool initializeCellularNetwork(unsigned long wakeUpCounter) {
  if (g_networkReady) {
    ESP_LOGI(TAG, "Network is already ready to use");
    return true;
  }

  if (g_ceAgSerial != nullptr || g_cellularCard != nullptr || g_agClient != nullptr) {
    ESP_LOGW(
        TAG,
        "Give up cellular initialization on this wakeup cycle since previous attempt is failed");
    return false;
  }

  // Enable CE card power
  gpio_set_level(EN_CE_CARD, 1);
  vTaskDelay(pdMS_TO_TICKS(100));

  if (wakeUpCounter == 0) {
    g_statusLed.set(StatusLed::Blink, 0, 1000);
  }

  g_ceAgSerial = new AirgradientUART();
  if (!g_ceAgSerial->begin(UART_BAUD_PORT_CE_CARD, UART_BAUD_CE_CARD, UART_RX_CE_CARD,
                           UART_TX_CE_CARD)) {
    ESP_LOGI(TAG, "Failed initialize serial communication for cellular card");
    g_statusLed.set(StatusLed::Blink, 1000, 100);
    return false;
  }

  // Enable debugging when CE card initializing
  g_ceAgSerial->setDebug(true);
  // Initialize cellular card and client
  g_cellularCard = new CellularModuleA7672XX(g_ceAgSerial, IO_CE_POWER);
  g_agClient = new AirgradientCellularClient(g_cellularCard);

  do {
    if (g_agClient->begin(g_serialNumber)) {
      // Connected
      if (wakeUpCounter == 0) {
        g_statusLed.set(StatusLed::Blink, 2000, 500);
      }
      break;
    }

    if (wakeUpCounter == 0) {
      ESP_LOGE(TAG, "Failed start airgradient client, retry in 10s");
      g_statusLed.set(StatusLed::Blink, 1000, 100);
      vTaskDelay(pdMS_TO_TICKS(10000));
      ESP_LOGI(TAG, "Retry starting airgradient client...");
      g_statusLed.set(StatusLed::Blink, 0, 1000);
    } else {
      ESP_LOGE(TAG, "Failed start airgradient client");
      break;
    }
  } while (wakeUpCounter == 0);
  // TODO: Add timeout to this loop and if failed just restart
  // TOOD: watchdog might reset on this loop

  // Disable again
  g_ceAgSerial->setDebug(false);
  g_networkReady = true;

  return true;
}

bool sendMeasuresWhenReady(unsigned long wakeUpCounter, PayloadCache &payloadCache) {
  // Check if pass criteria to post measures
  if (wakeUpCounter != 0 && (wakeUpCounter % TRANSMIT_MEASUREMENTS_CYCLES) > 0) {
    ESP_LOGI(TAG, "Not the time to post measures, skip");
    return true;
  }

  if (!initializeCellularNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to cellular network, skip send measures");
    return false;
  }

  // Push back signal same value to each payload
  auto result = g_cellularCard->retrieveSignal();
  int signalStrength = -1;
  if (result.status == CellReturnStatus::Ok && result.data != 99) {
    signalStrength = result.data;
  }
  ESP_LOGI(TAG, "Signal strength: %d", signalStrength);

  // Retrieve measurements from the cache
  std::vector<AirgradientClient::OpenAirMaxPayload> payloads;
  PayloadType tmp;
  ESP_LOGI(TAG, "cache size: %d", payloadCache.getSize());
  for (int i = 0; i < payloadCache.getSize(); i++) {
    payloadCache.peekAtIndex(i, &tmp);
    tmp.signal = signalStrength;
    payloads.push_back(tmp);
  }
  // TODO: Make sure send data success when wake up counter is 0

  // Attempt to send
  bool success = g_agClient->httpPostMeasures(g_remoteConfig.getConfigSchedule().pm02, payloads);
  if (!success) {
    // Consider network has a problem, retry in next schedule
    ESP_LOGE(TAG, "send measures failed, retry in next schedule");
    return false;
  }

  payloadCache.clean();

  return true;
}

bool checkForFirmwareUpdate(unsigned long wakeUpCounter) {
  if (wakeUpCounter != 0 && (wakeUpCounter % FIRMWARE_UPDATE_CHECK_CYCLES) > 0) {
    ESP_LOGI(TAG, "Not the time to check firmware update, skip");
    return true;
  }

  if (!initializeCellularNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to cellular network, skip check firmware update");
    return false;
  }

  AirgradientOTACellular agOta(g_cellularCard);
  auto result = agOta.updateIfAvailable(g_serialNumber, g_fimwareVersion);

  switch (result) {
  case AirgradientOTA::Failed:
    ESP_LOGI(TAG, "Firmware update failed");
    break;
  case AirgradientOTA::Skipped:
    ESP_LOGI(TAG, "Firmware update is skipped");
    break;
  case AirgradientOTA::AlreadyUpToDate:
    ESP_LOGI(TAG, "Firmware version already up to date");
    break;
  case AirgradientOTA::Success:
    ESP_LOGI(TAG, "Firmware update success, will restart in 3s...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();
    break;
  default:
    break;
  }

  return true;
}

bool checkRemoteConfiguration(unsigned long wakeUpCounter) {
  if (wakeUpCounter != 0 && (wakeUpCounter % FIRMWARE_UPDATE_CHECK_CYCLES) > 0) {
    ESP_LOGI(TAG, "Not the time to check remote configuration, skip");
    return true;
  }

  if (!initializeCellularNetwork(wakeUpCounter)) {
    ESP_LOGI(TAG, "Cannot connect to cellular network, skip check remote configuration");
    return false;
  }

  // Attempt retrieve configuration
  std::string result = g_agClient->httpFetchConfig();
  if (g_agClient->isRegisteredOnAgServer() == false) {
    ESP_LOGW(TAG, "Monitor hasn't registered on AirGradient dashboard yet");
    return false;
  }

  if (g_agClient->isLastFetchConfigSucceed() == false) {
    return false;
  }

  if (g_remoteConfig.parse(result) == false) {
    return false;
  }

  if (g_remoteConfig.isConfigChanged()) {
    ESP_LOGI(TAG, "Changed configuration will be applied on next wakeup cycle onwards");
  }

  return true;
}
