#include <stdio.h>
#include <inttypes.h>
#include "freertos/projdefs.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


static const char *const TAG = "APP";


extern "C" void app_main(void) {
  ESP_LOGI(TAG, "Hello world");


  while (1) {

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
