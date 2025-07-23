#include "AirgradientIICSerial.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include <cstring>

#define MAX_RETRY_IICSERIAL_UART_INIT 3

AirgradientIICSerial::AirgradientIICSerial(i2c_master_bus_handle_t i2c_bus_handle,
                                           uint8_t subUartChannel, uint8_t IA1, uint8_t IA0)
    : _iicSerial(i2c_bus_handle, subUartChannel, IA1, IA0) {}

bool AirgradientIICSerial::begin(int baud) { return begin(baud, -1); }

bool AirgradientIICSerial::begin(int baud, int iicResetIO) {
  if (isInitialized) {
    // already initialized
    ESP_LOGW(TAG, "IICSerial Already initialized");
    return true;
  }

  if (iicResetIO != -1) {
    // init iic serial
    _iicResetIO = static_cast<gpio_num_t>(iicResetIO);
    gpio_reset_pin(_iicResetIO); // IIC-UART reset
    gpio_set_direction(_iicResetIO, GPIO_MODE_OUTPUT);
    gpio_set_level(_iicResetIO, 1);
  }

  int counter = 0;
  bool opened = false;
  do {
    if (_iicSerial.begin(baud) == 0) {
      opened = true;
      break;
    }

    ESP_LOGW(TAG, "IICSerial failed open serial line, retry..");
    counter++;
    vTaskDelay(pdMS_TO_TICKS(500));
  } while (counter < MAX_RETRY_IICSERIAL_UART_INIT);

  if (!opened) {
    ESP_LOGE(TAG, "IICSerial failed open serial line, give up..");
    return false;
  }

  ESP_LOGI(TAG, "IICSerial success open serial line");
  isInitialized = true;
  return true;
}

void AirgradientIICSerial::end() {
  // TODO: Implement!
}

int AirgradientIICSerial::available() { return _iicSerial.available(); }

void AirgradientIICSerial::print(const char *str) {
  if (isDebug) {
#ifdef ARDUINO
    Serial.print(str);
#else
    // Prevent carriage return to stdout so its not go back to beginning of the line on webserial API
    // Specific for ATCommandHandler, it call this function for \r\n always in separate call
    if (strcmp(str, "\r\n") == 0) {
      printf("\n");
    } else {
      printf("%s", str);
    }
#endif
  }

  _iicSerial.print(str);
}

int AirgradientIICSerial::write(const uint8_t *data, int len) {
  //   if (isDebug) {
  // #ifdef ARDUINO
  //     Serial.print(str);
  // #else
  //     printf("%s", str);
  // #endif
  //   }

  return _iicSerial.write(data, len);
}

int AirgradientIICSerial::read() {
  if (isDebug) {
    char b = _iicSerial.read();
#ifdef ARDUINO
    Serial.write(b);
#else
    // Prevent carriage return to stdout so its not go back to beginning of the line on webserial API
    if (b != '\r') {
      printf("%c", b);
    }
#endif
    return b;
  }

  return _iicSerial.read();
}
