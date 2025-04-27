#include <cstdint>
#include <stdio.h>
#include <inttypes.h>
#include <string>

#include "esp_log_level.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#include "StatusLed.h"
#include "Sensor.h"


// Global Vars
static const char *const TAG = "APP";


// Prototype functions
static void enableIO(bool enableCECard);
static void resetExtWatchdog();
static void goSleep();


extern "C" void app_main(void) {
  ESP_LOGI(TAG, "MAX!");

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


  // TODO: Print out charging status
  // charger.getChargingStatus();


  while (1) {
    sensor.startMeasures(20, 2000);
    sensor.printMeasures();
    vTaskDelay(pdMS_TO_TICKS(3 * 60000));
    resetExtWatchdog();
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

void goSleep() {
  resetExtWatchdog();
  // TODO
}
