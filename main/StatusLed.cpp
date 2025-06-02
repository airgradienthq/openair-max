/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "StatusLed.h"
#include "driver/gpio.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "portmacro.h"

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

StatusLed::StatusLed(gpio_num_t ioLed) : _ioLed(ioLed) {}

StatusLed::~StatusLed() {
  // stop task
  stop();
}

esp_err_t StatusLed::start() {
  BaseType_t result;
  result = xTaskCreate(&StatusLed::_start, "StatusLed", 1024, this, 5, &_taskHandle);
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed start status led task");
    return ESP_FAIL;
  }

  // gpio_hold_dis(_ioLed);
  // gpio_reset_pin(_ioLed);
  gpio_set_direction(_ioLed, GPIO_MODE_OUTPUT);
  gpio_set_level(_ioLed, 0);

  ESP_LOGD(TAG, "Success start status led task");
  return ESP_OK;
}

void StatusLed::stop() {
  if (_isRunning) {
    vTaskDelete(_taskHandle);
  }
}

void StatusLed::disable() {
  gpio_set_level(_ioLed, 0);
  // gpio_hold_en(_ioLed);
}

void StatusLed::set(Mode mode, int durationMs, int intervalMs) {
  _lastDuration = durationMs;
  _lastMode = mode;
  _lastInterval = intervalMs;
  // Notify task there's a new led event
  xTaskNotifyGive(_taskHandle);
}

void StatusLed::_start(void *params) {
  StatusLed *pLed = static_cast<StatusLed *>(params);

  // This variable act as how long to wait for notification
  // And its also act as blink interval
  TickType_t waitNotificationTick = portMAX_DELAY;

  // waitNotificationTick = 100 / portTICK_PERIOD_MS;
  uint32_t notif;

  Mode mode;
  int duration = 0;
  int interval;

  int lastLedLevel = 0;
  int blinkStartTime = 0;

  while (1) {
    notif = ulTaskNotifyTake(pdTRUE, waitNotificationTick);
    if (notif > 0) {
      // New status led event received
      mode = pLed->_lastMode;
      duration = pLed->_lastDuration;
      interval = pLed->_lastInterval;

      // Handle start of the event based on each mode
      // For On and Off, no other action required, wait for event with blocking task
      switch (mode) {
      case Off:
        gpio_set_level(pLed->_ioLed, 0);
        waitNotificationTick = portMAX_DELAY;
        break;
      case On:
        gpio_set_level(pLed->_ioLed, 1);
        waitNotificationTick = portMAX_DELAY;
        break;
      case Blink:
        gpio_set_level(pLed->_ioLed, 1); // turn on first
        waitNotificationTick = interval / portTICK_PERIOD_MS;
        blinkStartTime = MILLIS();
        break;
      }

      continue;
    }

    // The rest is for blink mode

    // Check if duration is UP and duration is not set to forever (0)
    if ((MILLIS() - blinkStartTime) > duration && duration > 0) {
      gpio_set_level(pLed->_ioLed, 0);
      duration = 0;
      waitNotificationTick = portMAX_DELAY;
      continue;
    }

    lastLedLevel = !lastLedLevel;
    gpio_set_level(pLed->_ioLed, lastLedLevel);
  }
}
