#include <stdio.h>
#include <inttypes.h>
#include <string>
#include "AirgradientSerial.h"
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "AirgradientUART.h"
#include "driver/gpio.h"
#include "Sunlight.h"

static const gpio_num_t EN_CO2 = GPIO_NUM_15;
static const char *const TAG = "APP";
static const uint8_t CO2_SUNLIGHT_ADDR = 0x68;

extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Hello world");

  // Enable Sunlight
  gpio_reset_pin(EN_CO2); // IIC-UART reset
  gpio_set_direction(EN_CO2, GPIO_MODE_OUTPUT);
  gpio_set_level(EN_CO2, 1);

  AirgradientSerial *agSerial = new AirgradientUART;
  if (!agSerial->open(0, 9600, 0, 1)) {
    ESP_LOGE(TAG, "Failed open ag serial");
    while (1) {
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  Sunlight co2(*agSerial);

  vTaskDelay(pdMS_TO_TICKS(5000));
  co2.read_sensor_id(CO2_SUNLIGHT_ADDR);

  ESP_LOGI(TAG, "Forever loop");
  while (1) {
    auto measures = co2.read_sensor_measurements(CO2_SUNLIGHT_ADDR);
    ESP_LOGI(TAG, "CO2: %d", measures);
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}
