#include <stdio.h>
#include "PMS.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "esp_log.h"

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

PMS::PMS(AirgradientSerial *agSerial) : agSerial_(agSerial) {}

void PMS::sleep() {
  uint8_t command[] = {0x42, 0x4D, 0xE4, 0x00, 0x00, 0x01, 0x73};
  agSerial_->write(command, sizeof(command));
}

void PMS::wakeUp() {
  uint8_t command[] = {0x42, 0x4D, 0xE4, 0x00, 0x01, 0x01, 0x74};
  agSerial_->write(command, sizeof(command));
}

void PMS::activeMode() {
  uint8_t command[] = {0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71};
  agSerial_->write(command, sizeof(command));
  _mode = Mode::ACTIVE;
}

void PMS::passiveMode() {
  uint8_t command[] = {0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70};
  agSerial_->write(command, sizeof(command));
  _mode = Mode::PASSIVE;
}

bool PMS::isConnected() {
  bool connected = false;
  Data data;
  for (int i = 0; i < 3; i++) {
    clearBuffer();
    requestRead();
    if (readUntil(data, 1000)) {
      connected = true;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  return connected;
}

void PMS::clearBuffer() {
  // empty buffer first
  int bytesCleared = 0;
  while (uint8_t bb = agSerial_->read() != -1) {
    ESP_LOGD(TAG, "%.2x", bb);
    bytesCleared++;
  }
  ESP_LOGD(TAG, "Cleared %d byte(s)", bytesCleared);
}

void PMS::requestRead() {
  if (_mode == Mode::PASSIVE) {
    uint8_t command[] = {0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71};
    agSerial_->write(command, sizeof(command));
  }
}

bool PMS::read(Data &data) { return _loop(data); }

bool PMS::readUntil(Data &data, uint16_t timeoutMs) {
  uint32_t start = MILLIS();
  bool newData = false;
  do {
    if (_loop(data)) {
      newData = true;
      break;
    }
  } while ((MILLIS() - start) < timeoutMs);

  return newData;
}

bool PMS::_loop(Data &data) {
  bool dataReady = false;
  if (agSerial_->available()) {
    uint8_t ch = agSerial_->read();
    ESP_LOGV(TAG, "%.2x", ch);
    switch (_index) {
    case 0:
      // Reset checksum if fixed 0x42 start character arrives (start of stream).
      if (ch != 0x42) {
        return false;
      }
      _calculatedChecksum = ch;
      break;

    case 1:
      // Add fixed 0x4D second character to checksum, when it arrives.
      if (ch != 0x4D) {
        _index = 0;
        return false;
      }
      _calculatedChecksum += ch;
      break;

    case 2:
      // High byte of frame length, add to calculated checksum and shift out the byte to calculate the frame length.
      _calculatedChecksum += ch;
      _frameLen = ch << 8;
      break;

    case 3:
      // Low byte of frame length, add to calculated checksum through an inclusive OR to the frame length.
      _frameLen |= ch;
      // Unsupported sensor, different frame length, transmission error e.t.c.
      if (_frameLen != 2 * 9 + 2 && _frameLen != 2 * 13 + 2 && _frameLen != 2 * 17 + 2) {
        _index = 0;
        return false;
      }
      _calculatedChecksum += ch;
      break;

    default:
      // High byte of checksum, add to checksum and shift out the byte.
      if (_index == _frameLen + 2) {
        _checksum = ch << 8;
      } else if (_index == _frameLen + 2 + 1) {
        // Low byte of checksum, add to checksum through an inclusive OR.
        _checksum |= ch;

        // Data is valid and sane, safe to read.
        if (_calculatedChecksum == _checksum) {
          // _status = STATUS_OK;
          dataReady = true;

          // Standard Particles, CF=1.
          data.pm_sp_1_0 = _makeWord(_payload[0], _payload[1]);
          data.pm_sp_2_5 = _makeWord(_payload[2], _payload[3]);
          data.pm_sp_10_0 = _makeWord(_payload[4], _payload[5]);

          // Atmospheric Environment.
          data.pm_ae_1_0 = _makeWord(_payload[6], _payload[7]);
          data.pm_ae_2_5 = _makeWord(_payload[8], _payload[9]);
          data.pm_ae_10_0 = _makeWord(_payload[10], _payload[11]);

          // Total particles count per 100ml air
          data.pm_raw_0_3 = _makeWord(_payload[12], _payload[13]);
          data.pm_raw_0_5 = _makeWord(_payload[14], _payload[15]);
          data.pm_raw_1_0 = _makeWord(_payload[16], _payload[17]);
          data.pm_raw_2_5 = _makeWord(_payload[18], _payload[19]);
          data.pm_raw_5_0 = _makeWord(_payload[20], _payload[21]);
          data.pm_raw_10_0 = _makeWord(_payload[22], _payload[23]);

          // Temperature & humidity (PMSxxxxST units only)
          data.amb_temp = _makeWord(_payload[20], _payload[21]);
          data.amb_hum = _makeWord(_payload[22], _payload[23]);

          // Formaldehyde concentration (PMSxxxxST units only)
          data.amb_hcho = _makeWord(_payload[24], _payload[25]) / 1000;
        }

        _index = 0;
        return dataReady;
      } else {
        _calculatedChecksum += ch;
        uint8_t payloadIndex = _index - 4;

        // Payload is common to all sensors (first 2x6 bytes).
        if (payloadIndex < sizeof(_payload)) {
          _payload[payloadIndex] = ch;
        }
      }

      break;
    }

    _index++;
  } else {
    ESP_LOGD(TAG, "Stream not available");
  }

  return dataReady;
}

uint16_t PMS::_makeWord(uint8_t high, uint8_t low) { return (uint16_t)((high << 8) | low); }
