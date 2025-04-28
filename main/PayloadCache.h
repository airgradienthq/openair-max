#ifndef PAYLOAD_CACHE_H
#define PAYLOAD_CACHE_H

#include "airgradientClient.h"
#include <cstdint>
#include "config.h"

typedef AirgradientClient::OpenAirMaxPayload PayloadType;

class PayloadCache {
public:
  PayloadCache(uint16_t maxSize);
  ~PayloadCache();

  void restoreFromRTC();
  void backupToRTC();
  uint16_t getSize();

  void clean();
  void push(PayloadType *payload);
  bool pop(PayloadType *payload);
  bool peekAtIndex(uint16_t index, PayloadType *payload);

  void serializeQueueToCharArray();

private:
  const char* const TAG = "PayloadCache";
  uint16_t maxCacheSize;
  uint16_t head;                  // Head index for the queue
  uint16_t tail;                  // Tail index for the queue
  PayloadType payloads[MAX_PAYLOAD_CACHE];  // Buffer for Queue

  // Temporary buffer for serialization
  char charBuffer[sizeof(PayloadType) * MAX_PAYLOAD_CACHE];

  void serialize();
  void deserialize();
};

#endif
