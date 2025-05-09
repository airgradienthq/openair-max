/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef AIRGRADIENT_UART_H
#define AIRGRADIENT_UART_H

#include "AirgradientSerial.h"
#include "hal/uart_types.h"

class AirgradientUART : public AirgradientSerial {
public:
  AirgradientUART() {};
  ~AirgradientUART() {};

  bool begin(int port_num, int baud, int rx, int tx);
  void end();
  int available();
  void print(const char *str);
  int write(const uint8_t *data, int len);
  int read();

private:
  static constexpr int BUF_SIZE = 1024;
  uart_port_t _port_num = UART_NUM_0;
};

#endif // !AIRGRADIENT_UART_H
