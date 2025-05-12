/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef AIRGRADIENT_SERIAL_H
#define AIRGRADIENT_SERIAL_H

#include <stddef.h>
#include <stdint.h>

class AirgradientSerial {
public:
  AirgradientSerial();
  virtual ~AirgradientSerial();


  virtual bool begin(int baud);
  virtual bool begin(int baud, int iicResetIO);
  virtual bool begin(int port, int baud, int rx, int tx);
  virtual void end();
  virtual int available();
  virtual void print(const char *str);
  virtual int write(const uint8_t *data, int len);
  virtual int read();
  void setDebug(bool debug);

protected:
  const char *const TAG = "AGSerial";
  bool isInitialized = false;
  bool isDebug = false;
};

#endif // !AIRGRADIENT_SERIAL_H
