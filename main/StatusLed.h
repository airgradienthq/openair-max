#include "soc/gpio_num.h"
#ifndef STATUS_LED_H

#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"

class StatusLed {
public:
  enum Mode { Off, On, Blink };

  StatusLed(gpio_num_t ioLed);
  ~StatusLed();

  esp_err_t start();
  void stop();

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
  bool _isRunning = false;
  Mode _lastMode = Off;
  int _lastDuration = 0;
  int _lastInterval = 0;

  static void _start(void *params);
};
#endif // !STATUS_LED_H
