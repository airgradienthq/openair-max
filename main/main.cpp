#include <stdio.h>
#include <inttypes.h>
#include <string>
#include "AirgradientSerial.h"
#include "esp_log_level.h"
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

static const gpio_num_t EN_CO2 = GPIO_NUM_15;
static const gpio_num_t EN_PM1 = GPIO_NUM_3;
static const gpio_num_t EN_PM2 = GPIO_NUM_11;
static const gpio_num_t IO_WDT = GPIO_NUM_2;

static const char *const TAG = "APP";
static const uint8_t CO2_SUNLIGHT_ADDR = 0x68;

#define I2C_MASTER_SCL_IO 6
#define I2C_MASTER_SDA_IO 7
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_PORT 0

void reset() {
  gpio_set_level(IO_WDT, 1);
  vTaskDelay(pdMS_TO_TICKS(20));
  gpio_set_level(IO_WDT, 0);
}

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Hello world");

  gpio_reset_pin(EN_PM1);
  gpio_set_direction(EN_PM1, GPIO_MODE_OUTPUT);
  gpio_set_level(EN_PM1, 1);
  gpio_reset_pin(EN_PM2);
  gpio_set_direction(EN_PM2, GPIO_MODE_OUTPUT);
  gpio_set_level(EN_PM2, 1);

  // watchdog
  gpio_reset_pin(IO_WDT);
  gpio_set_direction(IO_WDT, GPIO_MODE_OUTPUT);
  gpio_set_level(IO_WDT, 0);

  // // Enable Sunlight
  // gpio_reset_pin(EN_CO2);
  // gpio_set_direction(EN_CO2, GPIO_MODE_OUTPUT);
  // gpio_set_level(EN_CO2, 1);
  //
  // AirgradientSerial *agSerial = new AirgradientUART;
  // if (!agSerial->open(0, 9600, 0, 1)) {
  //   ESP_LOGE(TAG, "Failed open ag serial");
  //   while (1) {
  //     vTaskDelay(pdMS_TO_TICKS(1000));
  //   }
  // }

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

  AirgradientSerial *agSerial = new AirgradientIICSerial(bus_handle, SUBUART_CHANNEL_1, 0, 1);
  if (agSerial->begin(9600) != 0) {
    ESP_LOGE(TAG, "Failed open ag serial");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  // Sunlight co2(*agSerial);
  // vTaskDelay(pdMS_TO_TICKS(5000));
  // co2.read_sensor_id(CO2_SUNLIGHT_ADDR);

  // vTaskDelay(pdMS_TO_TICKS(1000));

  PMS pms(agSerial);

  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Forever loop");
  bool warmup = true;
  if (warmup) {
    for (int i = 10; i >= 0; i--) {
      ESP_LOGI(TAG, "Warming up PMS sensors %d", i);
      vTaskDelay(pdMS_TO_TICKS(1000));
      pms.passiveMode();
    }
  }

  // agSerial->flush();

  while (1) {
    // auto measures = co2.read_sensor_measurements(CO2_SUNLIGHT_ADDR);
    // ESP_LOGI(TAG, "CO2: %d", measures);
    // vTaskDelay(pdMS_TO_TICKS(5000));

    pms.requestRead();
    PMS::Data data;
    if (pms.readUntil(data, 3000)) {
      ESP_LOGI(TAG, "PM1.0 = %d", data.pm_ae_1_0);
      ESP_LOGI(TAG, "PM2.5 = %d", data.pm_ae_2_5);
      ESP_LOGI(TAG, "PM10.0 = %d", data.pm_ae_10_0);
      ESP_LOGI(TAG, "PM 0.3 count = %d", data.pm_raw_0_3);
    } else {
      ESP_LOGW(TAG, "No data");
    }
    reset();
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
