
/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef AIRGRADIENT_IICSERIAL_H
#define AIRGRADIENT_IICSERIAL_H

#include "AirgradientSerial.h"
#include "DFRobot_IICSerial.h"

class AirgradientIICSerial : public AirgradientSerial {
public:
  AirgradientIICSerial(i2c_master_bus_handle_t i2c_bus_handle,
                       uint8_t subUartChannel = SUBUART_CHANNEL_1, uint8_t IA1 = 1,
                       uint8_t IA0 = 1);
  ~AirgradientIICSerial() {};

  bool begin(int baud, int iicResetIO);
  void end();
  int available();
  void print(const char *str);
  int write(const uint8_t *data, int len);
  int read();

private:
  DFRobot_IICSerial _iicSerial;
  gpio_num_t _iicResetIO = GPIO_NUM_NC;
};

#endif // !AIRGRADIENT_IICSERIAL_H
