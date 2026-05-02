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
  static constexpr int DBG_LINE_BUF = 256;
  uart_port_t _port_num = UART_NUM_0;

  // Line buffers for debug mirror so AT TX/RX traffic emits via ESP_LOG and
  // therefore reaches both USB-CDC and the UDP debug logger. Each direction
  // accumulates bytes until '\n' or buffer overflow, then flushes one log line.
  char _dbgTxBuf[DBG_LINE_BUF];
  size_t _dbgTxLen = 0;
  char _dbgRxBuf[DBG_LINE_BUF];
  size_t _dbgRxLen = 0;
  void _flushDebugLine(char *buf, size_t &len, const char *tag);
  void _appendDebug(char *buf, size_t &len, const char *tag, const char *src,
                    size_t srcLen);
};

#endif // !AIRGRADIENT_UART_H
