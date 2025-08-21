/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "soc/gpio_num.h"
#ifndef STATUS_LED_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

class StatusLed {
public:
  enum Mode { Off, On, Blink };

  struct Status {
    Mode mode;
    int duration;
    int interval;
  };

  StatusLed(gpio_num_t ioLed);
  ~StatusLed();

  esp_err_t start();
  void stop();
  void holdState();
  void releaseState();

  void on();
  void off();
  void blink(int durationMs = 0, int intervalMs = 0);
  void blinkAsync(int durationMs = 0, int intervalMs = 0);

  /**
   * @brief set led notification based on provided Mode
   *
   * @param mode what mode to set to led notification
   * @param durationMs blink duration; If 0, then blink is set to forever until Set() called again
   * @param intervalMs interval between each blink
   */
  void set(Mode mode, int durationMs = 0, int intervalMs = 0);

private:
  const char *const TAG = "StatusLed";

  gpio_num_t _ioLed = GPIO_NUM_MAX;
  TaskHandle_t _taskHandle = NULL;
  bool _isRunning = false; // TODO: Handle better
  Status _newStatus;

  static void _start(void *params);
};
#endif // !STATUS_LED_H
