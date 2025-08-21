/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#ifndef PAYLOAD_CACHE_H
#define PAYLOAD_CACHE_H

#include <cstdint>
#include "airgradientClient.h"
#include "MaxConfig.h"

typedef AirgradientClient::MaxSensorPayload PayloadCacheType;

class PayloadCache {
public:
  PayloadCache(uint16_t maxSize);
  ~PayloadCache();

  void restoreFromRTC();
  void backupToRTC();
  uint16_t getSize();

  void clean();
  void push(PayloadCacheType *payload);
  bool pop(PayloadCacheType *payload);
  bool peekAtIndex(uint16_t index, PayloadCacheType *payload);

  void serializeQueueToCharArray();

private:
  const char* const TAG = "PayloadCache";
  uint16_t maxCacheSize;
  uint16_t head;                  // Head index for the queue
  uint16_t tail;                  // Tail index for the queue
  PayloadCacheType payloads[MAX_PAYLOAD_CACHE];  // Buffer for Queue

  // Temporary buffer for serialization
  // TODO: Need to optimize this, why the need to have 2 buffer?
  char charBuffer[sizeof(PayloadCacheType) * MAX_PAYLOAD_CACHE];

  void serialize();
  void deserialize();
};

#endif
