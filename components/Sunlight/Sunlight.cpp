#include "Sunlight.h"
#include <cstring>
#include "esp_log.h"
#include "esp_timer.h"

#define CO2_SUNLIGHT_ADDR 0x68

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))
#define DBG(...)

// From Arduino.h
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))

Sunlight::Sunlight(AirgradientSerial &serial) : _agSerial(serial) {}

int Sunlight::modbus_read_response(int waitBytes, uint8_t funCode) {
  /* Time-out variable */
  unsigned long byteTime = MILLIS();
  int available_bytes;
  unsigned long timestamp;
  /* Return variable */
  int error;

  /* Wait for first byte in packet */
  while ((available_bytes = _agSerial.available()) == 0) {
    unsigned long timeout = (unsigned long)((long)MILLIS() - (long)byteTime);
    if (WAIT_MS < timeout) {
      return COMMUNICATION_ERROR;
    }
  }

  byteTime = MILLIS();

  do {
    int new_available_bytes = _agSerial.available();

    timestamp = MILLIS();

    if (available_bytes != new_available_bytes) {
      byteTime = timestamp;
      available_bytes = new_available_bytes;
    }
  } while (INTER_PACKET_INTERVAL_MS > (unsigned long)((long)timestamp - (long)byteTime));

  for (int n = 0; n < available_bytes; n++) {
    response[n] = _agSerial.read();
  }

  /* Check response for exceptions */
  error = _handler(response, funCode, available_bytes);
  return ((error == 0) ? available_bytes : error);
}

int Sunlight::read_holding_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg) {
  /* Return variable */
  int error;

  /* PDU variables */
  uint8_t funCode = 0x03;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;

  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;

  /* Define Modbus PDU */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = regAddrHi;
  request[3] = regAddrLo;
  request[4] = numRegHi;
  request[5] = numRegLo;

  /* Create CRC */
  uint16_t crc = _generate_crc(request, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  request[6] = crcLo;
  request[7] = crcHi;

  /* Send request */
  _agSerial.write(request, 8);

  /* Number of bytes to wait for */
  int waitBytes = 5 + (numReg * 2);
  /* Wait for response */
  error = modbus_read_response(waitBytes, funCode);

  /* If no error were encountered, combine the bytes containing the requested values into words */
  if (error > 0) {
    int counter = 0;
    int slot = 3;
    while (counter < ((error - 5) / 2)) {
      values[counter] = ((int16_t)(int8_t)response[slot] << 8) | (uint16_t)response[slot + 1];

      counter++;
      slot = slot + 2;
    }
  } else {
    return error;
  }

  return 0;
}

int Sunlight::read_input_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg) {
  /* Return variable */
  int error = 0;
  /* PDU variables */
  uint8_t funCode = 0x04;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;

  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;

  /* Define Modbus PDU */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = regAddrHi;
  request[3] = regAddrLo;
  request[4] = numRegHi;
  request[5] = numRegLo;

  /* Create CRC */
  uint16_t crc = _generate_crc(request, 6);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  request[6] = crcLo;
  request[7] = crcHi;

  /* Send request */
  _agSerial.write(request, 8);

  /* Number of bytes to wait for */
  int waitBytes = 5 + (numReg * 2);
  /* Wait for response */
  error = modbus_read_response(waitBytes, funCode);

  /* If no error were encountered, combine the bytes containing the requested values into words */
  if (error > 0) {
    int counter = 0;
    int slot = 3;
    while (counter < ((error - 5) / 2)) {
      values[counter] = ((int16_t)(int8_t)response[slot] << 8) | (uint16_t)response[slot + 1];

      counter++;
      slot = slot + 2;
    }
  } else {
    return error;
  }

  return 0;
}

int Sunlight::write_multiple_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg,
                                       uint16_t writeVal[]) {
  /* Return variable */
  int error = 0;

  /* PDU variables */
  uint8_t funCode = 0x10;

  uint8_t regAddrHi = (regAddr >> 8);
  uint8_t regAddrLo = regAddr & 0xFF;

  uint8_t numRegHi = (numReg >> 8);
  uint8_t numRegLo = numReg & 0xFF;

  uint8_t numBytes = numReg * 2;

  /* Check if request fits in buffer */
  if (numBytes >= 249) {
    return -1;
  }

  uint8_t writeValHi;
  uint8_t writeValLo;

  int requestSize = 7 + numBytes;

  /* Assign the first 7 bytes to the request array */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = regAddrHi;
  request[3] = regAddrLo;
  request[4] = numRegHi;
  request[5] = numRegLo;
  request[6] = numBytes;

  /* Convert the words to be written into 2 bytes and assign them to the request array */
  int counter = 7;
  for (int n = 0; n < numBytes; n++) {
    request[counter] = (writeVal[n] >> 8);
    request[counter + 1] = writeVal[n] & 0xFF;
    counter += 2;
  }

  /* Create CRC */
  uint16_t crc = _generate_crc(request, requestSize);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  /* Request (HEX) */
  request[requestSize] = crcLo;
  request[requestSize + 1] = crcHi;

  requestSize += 2;

  _agSerial.write(request, requestSize);

  /* Number of bytes to wait for */
  int waitBytes = 8;
  /* Wait for response */
  error = modbus_read_response(waitBytes, funCode);

  return (error > 0) ? 0 : error;
}

int Sunlight::read_device_id(uint8_t comAddr, uint8_t objId) {
  memset(device, 0, 14);
  /* Return variable */
  int error = 0;
  /* PDU variables */
  uint8_t funCode = 0x2B;
  uint8_t meiType = 0x0E;
  uint8_t idCode = 0x04;

  /* Define Modbus PDU */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = meiType;
  request[3] = idCode;
  request[4] = objId;

  /* Create CRC */
  uint16_t crc = _generate_crc(request, 5);
  uint8_t crcLo = crc & 0xFF;
  uint8_t crcHi = (crc >> 8);

  request[5] = crcLo;
  request[6] = crcHi;

  /* Send request */
  _agSerial.write(request, 7);

  /* Number of bytes to wait for */
  int objLen = 0;
  if (objId == 0) {
    objLen = 8;
  } else if (objId == 1) {
    objLen = 7;
  } else if (objId == 2) {
    objLen = 4;
  }
  int waitBytes = 12 + objLen;
  /* Wait for response */
  error = modbus_read_response(waitBytes, funCode);
  if (error > 0) {
    /* Combine the bytes containing the requested values into words */
    int objLength = response[9];
    int slot = 10;
    for (int n = 0; n < objLength; n++) {
      device[n] = response[slot];

      slot++;
    }
    device[objLength] = '\0';
    return 0;
  } else {
    return error;
  }
}

int Sunlight::_handler(uint8_t pdu[], uint8_t funCode, int len) {
  /* Return variable */
  int error = 0;
  /* Function variables */
  uint8_t exceptionFunCode = funCode + 0x80;
  /* Check for malformed packet */
  if (len >= 4) {
    /* Check for corrupt data in the response */
    uint16_t crc = _generate_crc(pdu, (len - 2));
    uint8_t crcHi = (crc >> 8);
    uint8_t crcLo = crc & 0xFF;

    if (crcLo != pdu[len - 2] || crcHi != pdu[len - 1]) {
      return COMMUNICATION_ERROR;
    }

    /* Check response for exceptions */
    if (pdu[1] == exceptionFunCode) {
      switch (pdu[2]) {
      case ILLEGAL_FUNCTION:
        error = -ILLEGAL_FUNCTION;
        break;

      case ILLEGAL_DATA_ADDRESS:
        error = -ILLEGAL_DATA_ADDRESS;
        break;

      case ILLEGAL_DATA_VALUE:
        error = -ILLEGAL_DATA_VALUE;
        break;

      default:
        error = COMMUNICATION_ERROR;
        break;
      }
    }
  } else {
    error = COMMUNICATION_ERROR;
  }
  return error;
}

uint16_t Sunlight::_generate_crc(uint8_t pdu[], int len) {
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    /* XOR the byte into the least significant byte of crc */
    crc ^= (uint16_t)pdu[pos];

    /* Loop through the entire message */
    for (int n = 8; n != 0; n--) {
      /* If the LSB is 1, shift right and XOR 0xA001 */
      /* Otherwise, just shift right */
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

void Sunlight::read_sensor_config() {
  /* Function variables */
  uint16_t numReg = 0x0003;

  if (read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_MODE, numReg) != 0) {
    ESP_LOGE(TAG, "Failed to read Sensor Configurations");
    return;
  }

  // Printout sensor configurations
  uint16_t measMode = values[0];
  uint16_t measPeriod = values[1];
  uint16_t measSamples = values[2];
  ESP_LOGI(TAG, "Measurement mode: %d", measMode);
  ESP_LOGI(TAG, "Measurement period: %d sec", measPeriod);
  ESP_LOGI(TAG, "Number of samples: %d", measSamples);
}

bool Sunlight::set_measurement_mode(uint8_t mode) {
  /* Function variables */
  uint16_t numReg = 0x0001;
  uint16_t change[] = {CONTINUOUS};

  if (mode == SINGLE) {
    change[0] = SINGLE;
  }

  if (read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_MODE, numReg) != 0) {
    ESP_LOGE(TAG, "Failed to read Measurement Mode");
    ESP_LOGE(TAG, "Failed to change Measurement Mode");
    // TODO: handle better
    /* FATAL ERROR */
    while (true)
      ;
  }

  if (values[0] != change[0]) {
    if (mode == CONTINUOUS) {
      ESP_LOGI(TAG, "Changing Measurement Mode to Continuous...");
    } else {
      ESP_LOGI(TAG, "Changing Measurement Mode to Single...");
    }

    if (write_multiple_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_MODE, numReg, change) != 0) {
      ESP_LOGE(TAG, "Failed to change measurement mode");
      // TODO: handle better
      /* FATAL ERROR */
      while (true)
        ;
    }
    ESP_LOGW(TAG, "Sensor restart is required to apply changes");
    return true;
  }
  return false;
}

bool Sunlight::set_measurement_period(uint16_t seconds) {
  /* Function variables */
  uint16_t numReg = 0x0001;
  uint16_t change[] = {seconds};

  int error = read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_PERIOD, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read measurement period (%d)", error);
    ESP_LOGE(TAG, "Failed to change Measurement period");
    // TODO: handle better
    /* FATAL ERROR */
    while (true)
      ;
  }

  if (values[0] != seconds) {
    ESP_LOGI(TAG, "Changing measurement period to %ds", seconds);
    if (write_multiple_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_PERIOD, numReg, change) != 0) {
      ESP_LOGE(TAG, "Failed to change measurement period");
      /* FATAL ERROR */
      while (true)
        ;
    }
    ESP_LOGI(TAG, "Sensor restart is required to apply changes");
    return true;
  }
  return false;
}

bool Sunlight::set_measurement_samples(uint16_t number) {
  /* Function variables */
  uint16_t numReg = 0x0001;
  uint16_t change[] = {number};

  int error = read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_SAMPLES, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read measurement samples (%d)", error);
    ESP_LOGE(TAG, "Failed to change measurement samples");
    /* FATAL ERROR */
    // TODO: handle better
    while (true)
      ;
  }

  if (values[0] != number) {
    ESP_LOGI(TAG, "Changing measurement samples to %d", number);
    if (write_multiple_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_SAMPLES, numReg, change) != 0) {
      ESP_LOGE(TAG, "Failed to change Measurement samples");
      /* FATAL ERROR */
      while (true)
        ;
    }

    ESP_LOGI(TAG, "Sensor restart is required to apply changes");
    return true;
  }
  return false;
}

int16_t Sunlight::read_sensor_measurements() {
  /* Function variables */
  int error;
  uint16_t numReg = 0x0004;
  uint16_t co2Value = 0;

  /* Read values */
  if ((error = read_input_registers(CO2_SUNLIGHT_ADDR, ERROR_STATUS, numReg)) != 0) {
    ESP_LOGE(TAG, "Failed to read input register (%d)", error);
  } else {
    /* Read CO2 concentration */
    co2Value = (int16_t)values[3];

    /* Read error status */
    ESP_LOGD(TAG, "Error Status: (0x%04x)", values[0]);
  }

  return co2Value;
}

void Sunlight::read_sensor_id() {
  /* Vendor Name */
  if (read_device_id(CO2_SUNLIGHT_ADDR, 0) != 0) {
    ESP_LOGE(TAG, "Failed to read vendor name");
    return;
  }

  ESP_LOGI(TAG, "Vendor name: %s", device);

  /* ProductCode */
  if (read_device_id(CO2_SUNLIGHT_ADDR, 1) != 0) {
    ESP_LOGE(TAG, "Failed to read product code");
    return;
  }

  ESP_LOGI(TAG, "Product code: %s", device);

  /* MajorMinorRevision */
  if (read_device_id(CO2_SUNLIGHT_ADDR, 2) != 0) {
    ESP_LOGE(TAG, "Failed to read MajorMinorRevision");
    return;
  }

  ESP_LOGI(TAG, "MajorMinorRevision: %s", device);
}

bool Sunlight::isABCEnabled() {
  /* Function variables */
  uint16_t numReg = 0x0001;

  int error = read_holding_registers(CO2_SUNLIGHT_ADDR, METER_CONTROL, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read meter control (%d)", error);
    // TODO: handle better
    /* FATAL ERROR */
    while (true)
      ;
  }

  return (bitRead(values[0], 1) == 0);
}

bool Sunlight::setMeterControlBit(uint8_t target, bool newValue, uint8_t bit) {
  /* Function variables */
  uint16_t numReg = 0x0001;

  int error = read_holding_registers(target, METER_CONTROL, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read meter control (%d)", error);
    // TODO: Handle better
    /* FATAL ERROR */
    while (true)
      ;
  }
  uint16_t meterControlVal = values[0];
  bool isHigh = bitRead(meterControlVal, bit) == 0;

  if (isHigh != newValue) {
    /* Not match change it */
    if (newValue) {
      bitClear(meterControlVal, bit);
    } else {
      bitSet(meterControlVal, bit);
    }
    uint16_t change[] = {meterControlVal};
    if (write_multiple_registers(target, METER_CONTROL, numReg, change) != 0) {
      ESP_LOGE(TAG, "Failed to change ABC status");
      // TODO: handle better
      /* FATAL ERROR */
      while (true)
        ;
    }
    return true;
  }

  return false;
}

void Sunlight::setABC(bool enable) {
  if (!this->setMeterControlBit(CO2_SUNLIGHT_ADDR, enable, 1)) {
    DBG("ABC value change not needed");
  }
}
void Sunlight::setNRDY(bool enable) {
  if (!this->setMeterControlBit(CO2_SUNLIGHT_ADDR, enable, 0)) {
    DBG("nRDY value change not needed");
  }
}
