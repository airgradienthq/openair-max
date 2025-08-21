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
  // Set default value for led notification when starting
  _newStatus.duration = 0;
  _newStatus.interval = 0;
  _newStatus.mode = Off;

  BaseType_t result;
  result = xTaskCreate(&StatusLed::_start, "StatusLed", 1024, this, 5, &_taskHandle);
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed start status led task");
    return ESP_FAIL;
  }
  _isRunning = true;

  gpio_set_direction(_ioLed, GPIO_MODE_OUTPUT);
  gpio_set_level(_ioLed, 0);

  ESP_LOGD(TAG, "Success start status led task");
  return ESP_OK;
}

void StatusLed::stop() {
  if (_isRunning) {
    vTaskDelete(_taskHandle);
    _isRunning = false;
  }
}

void StatusLed::holdState() { gpio_hold_en(_ioLed); }

void StatusLed::releaseState() { gpio_hold_dis(_ioLed); }

void StatusLed::on() { set(On); }

void StatusLed::off() { set(Off); }

void StatusLed::blinkAsync(int durationMs, int intervalMs) { set(Blink, durationMs, intervalMs); }

void StatusLed::blink(int durationMs, int intervalMs) {
  set(Blink, durationMs, intervalMs);
  vTaskDelay(pdMS_TO_TICKS(durationMs));
}

void StatusLed::set(Mode mode, int durationMs, int intervalMs) {
  _newStatus.mode = mode;
  _newStatus.duration = durationMs;
  _newStatus.interval = intervalMs;
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

  // Set default values
  Status lastStatus = pLed->_newStatus;
  Status currentStatus = lastStatus;

  int lastLedLevel = 0;
  int blinkStartTime = 0;

  while (1) {
    notif = ulTaskNotifyTake(pdTRUE, waitNotificationTick);
    if (notif > 0) {
      // New status led event received

      // Keep the last status for later to get back to
      lastStatus = currentStatus;

      // Set the current status
      currentStatus = pLed->_newStatus;

      // Handle start of the event based on each mode
      // For On and Off, no other action required, wait for event with blocking task
      switch (currentStatus.mode) {
      case Off:
        gpio_set_level(pLed->_ioLed, 0);
        waitNotificationTick = portMAX_DELAY;
        lastLedLevel = 0;
        break;
      case On:
        gpio_set_level(pLed->_ioLed, 1);
        waitNotificationTick = portMAX_DELAY;
        lastLedLevel = 1;
        break;
      case Blink:
        lastLedLevel = !lastLedLevel;
        gpio_set_level(pLed->_ioLed, lastLedLevel);
        waitNotificationTick = currentStatus.interval / portTICK_PERIOD_MS;
        blinkStartTime = MILLIS();
        vTaskDelay(5);
        break;
      }

      continue;
    }

    // The rest is for blink mode

    // Check if duration is UP and duration is not set to forever (0)
    if ((MILLIS() - blinkStartTime) >= currentStatus.duration && currentStatus.duration > 0) {
      if (lastStatus.duration == 0 && lastStatus.mode == Blink) {
        // If last status duration expected to forever, go back to last status after this animation
        currentStatus = lastStatus;

        // Set starting level negate from current level
        lastLedLevel = !lastLedLevel;
        gpio_set_level(pLed->_ioLed, lastLedLevel);

        waitNotificationTick = currentStatus.interval / portTICK_PERIOD_MS;
        blinkStartTime = MILLIS();
      }

      if (lastStatus.mode == On) {
        gpio_set_level(pLed->_ioLed, 1);
        lastLedLevel = 1;
      } else {
        gpio_set_level(pLed->_ioLed, 0);
        lastLedLevel = 0;
      }
      currentStatus.mode = lastStatus.mode;
      currentStatus.duration = 0;
      waitNotificationTick = portMAX_DELAY;
      continue;
    }

    lastLedLevel = !lastLedLevel;
    gpio_set_level(pLed->_ioLed, lastLedLevel);
  }
}
