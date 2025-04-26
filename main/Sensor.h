#ifndef AG_SENSOR_H
#define AG_SENSOR_H

#include "AirgradientIICSerial.h"
#include "AirgradientUART.h"
#include "BQ25672.h"
#include "PMS.h"
#include "Sunlight.h"
#include "airgradientClient.h"
#include "sht4x.h"
#include "sgp4x.h"


class Sensor {
public:
  Sensor(i2c_master_bus_handle_t busHandle);
  ~Sensor(){}
  bool init();
  bool startMeasure(int signalStrength, int iterations, int intervalMs);

  // TODO: Caching
  // getMeasuresCachedSize()
  // getMeasuresCached(pointer to a memory?)
  // Save to cache, need to add itertion interval and signal strenght to pass here

private:
  const char *const TAG = "Sensor";

  void _measure(AirgradientClient::OpenAirMaxPayload &data);
  void _warmUpSGP41();
  void _warmUpPMS();

  AirgradientClient::OpenAirMaxPayload _averageMeasure;
  i2c_master_bus_handle_t _busHandle;

  bool _co2Available = true;
  AirgradientSerial *agsCO2_ = nullptr;
  Sunlight *co2_ = nullptr;

  bool _pms1Available = true;
  AirgradientSerial *agsPM1_ = nullptr;
  PMS *pms1_ = nullptr;

  bool _pms2Available = true;
  AirgradientSerial *agsPM2_ = nullptr;
  PMS *pms2_ = nullptr;

  bool _chargerAvailable = true;
  BQ25672 *charger_ = nullptr;

  bool _tempHumAvailable = true;
  sht4x_handle_t sht_dev_hdl;

  bool _tvocNoxAvailable = true;
  sgp4x_handle_t sgp_dev_hdl;

};

#endif // !AG_SENSOR_H
