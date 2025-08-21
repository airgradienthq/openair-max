/**
 * AirGradient
 * https://airgradient.com
 *
 * CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License
 */

#include "PayloadCache.h"

#include <cstring>

#include "esp_system.h"
#include "esp_log.h"

// Store serialized data in RTC Memory
RTC_DATA_ATTR char rtcStorage[sizeof(PayloadCacheType) * MAX_PAYLOAD_CACHE];
RTC_DATA_ATTR uint16_t rtcHead;
RTC_DATA_ATTR uint16_t rtcTail;

PayloadCache::PayloadCache(uint16_t maxSize) {
  maxCacheSize = maxSize;
  head = 0;
  tail = 0;

  // If there's data in RTC, restore it for use
  restoreFromRTC();
}

PayloadCache::~PayloadCache() {
  // Backup data to RTC before destroying the object
  backupToRTC();
}

void PayloadCache::restoreFromRTC() {
  // Copy data from RTC storage to charBuffer
  memcpy(charBuffer, rtcStorage, sizeof(charBuffer));
  head = rtcHead;
  tail = rtcTail;
  deserialize(); // Convert back to Queue
}

void PayloadCache::backupToRTC() {
  serialize(); // Convert Queue to char array
  // Copy data from charBuffer to RTC storage
  memcpy(rtcStorage, charBuffer, sizeof(charBuffer));
  rtcHead = head;
  rtcTail = tail;
}

void PayloadCache::clean() {
  head = 0;
  tail = 0;
  backupToRTC(); // Save changes to RTC
}

void PayloadCache::push(PayloadCacheType *payload) {
  payloads[tail] = *payload;
  tail = (tail + 1) % maxCacheSize;
  if (tail == head) { // Queue overflow, move head
    head = (head + 1) % maxCacheSize;
  }
  backupToRTC(); // Save changes to RTC
}

bool PayloadCache::pop(PayloadCacheType *payload) {
  if (head == tail) {
    return false; // Queue is empty
  }
  *payload = payloads[head];
  head = (head + 1) % maxCacheSize;
  backupToRTC(); // Save changes to RTC
  return true;
}

bool PayloadCache::peekAtIndex(uint16_t index, PayloadCacheType *payload) {
  if (index >= getSize()) {
    return false; // Index out of range
  }
  uint16_t actualIndex = (head + index) % maxCacheSize;
  *payload = payloads[actualIndex];
  return true;
}

uint16_t PayloadCache::getSize() {
  if (tail >= head) {
    return tail - head;
  } else {
    return maxCacheSize - head + tail;
  }
}

void PayloadCache::serializeQueueToCharArray() {
  // Serialize the current queue into charBuffer
  memset(charBuffer, 0, sizeof(charBuffer)); // Clear buffer
  memcpy(charBuffer, payloads, sizeof(payloads));

  // Print serialized char array
  // ESP_LOGI(TAG, "Serialized Queue as Char Array");
  // for (size_t i = 0; i < sizeof(charBuffer); i++) {
  //   Serial.print(charBuffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
}

void PayloadCache::serialize() {
  // Copy data from payloads to charBuffer as raw bytes
  memcpy(charBuffer, payloads, sizeof(payloads));
}

void PayloadCache::deserialize() {
  // Copy data from charBuffer to payloads as raw bytes
  memcpy(payloads, charBuffer, sizeof(payloads));
}
