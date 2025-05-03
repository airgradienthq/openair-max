#ifndef PMS_H
#define PMS_H

#include "AirgradientSerial.h"
class PMS {
public:
  struct Data {
    // Standard Particles, CF=1
    uint16_t pm_sp_1_0;
    uint16_t pm_sp_2_5;
    uint16_t pm_sp_10_0;

    // Atmospheric environment
    uint16_t pm_ae_1_0;
    uint16_t pm_ae_2_5;
    uint16_t pm_ae_10_0;

    // Raw particles count (number of particles in 0.1l of air
    uint16_t pm_raw_0_3;
    uint16_t pm_raw_0_5;
    uint16_t pm_raw_1_0;
    uint16_t pm_raw_2_5;
    uint16_t pm_raw_5_0;
    uint16_t pm_raw_10_0;

    // Formaldehyde (HCHO) concentration in mg/m^3 - PMSxxxxST units only
    uint16_t amb_hcho;

    // Temperature & humidity - PMSxxxxST units only
    int16_t amb_temp;
    uint16_t amb_hum;
  };

  PMS(AirgradientSerial *agSerial);
  ~PMS() {}

  // Standby mode. For low power consumption and prolong the life of the sensor.
  void sleep();
  // Operating mode. Stable data should be got at least 30 seconds after the sensor wakeup from the sleep mode because of the fan's performance.
  void wakeUp();
  // Active mode. Default mode after power up. In this mode sensor would send serial data to the host automatically.
  void activeMode();
  // Passive mode. In this mode sensor would send serial data to the host only for request.
  void passiveMode();

  bool isConnected();

  void clearBuffer();
  // Request read in Passive Mode.
  void requestRead();
  // Non-blocking function for parse response.
  bool read(Data &data);
  // Blocking function for parse response. Default timeout is 1s.
  bool readUntil(Data &data, uint16_t timeoutMs = 1000);

private:
  const char *const TAG = "PMS";
  AirgradientSerial *agSerial_ = nullptr;
  enum class Mode { ACTIVE, PASSIVE };
  Mode _mode = Mode::ACTIVE;

  uint8_t _payload[30];

  uint8_t _index = 0;
  uint16_t _frameLen;
  uint16_t _checksum;
  uint16_t _calculatedChecksum;

  bool _loop(Data &data);
  uint16_t _makeWord(uint8_t high, uint8_t low);
};

#endif // PMS_H
