#include "Sunlight.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include <cstring>

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
  } while (INTER_PACKET_INTERVAL_MS >
           (unsigned long)((long)timestamp - (long)byteTime));

  for (int n = 0; n < available_bytes; n++) {
    response[n] = _agSerial.read();
  }

  /* Check response for exceptions */
  error = _handler(response, funCode, available_bytes);
  return ((error == 0) ? available_bytes : error);
}

int Sunlight::read_holding_registers(uint8_t comAddr, uint16_t regAddr,
                                     uint16_t numReg) {
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

  /* If no error were encountered, combine the bytes containing the requested
   * values into words */
  if (error > 0) {
    int counter = 0;
    int slot = 3;
    while (counter < ((error - 5) / 2)) {
      values[counter] =
          ((int16_t)(int8_t)response[slot] << 8) | (uint16_t)response[slot + 1];

      counter++;
      slot = slot + 2;
    }
  } else {
    return error;
  }

  return 0;
}

int Sunlight::read_input_registers(uint8_t comAddr, uint16_t regAddr,
                                   uint16_t numReg) {
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

  /* If no error were encountered, combine the bytes containing the requested
   * values into words */
  if (error > 0) {
    int counter = 0;
    int slot = 3;
    while (counter < ((error - 5) / 2)) {
      values[counter] =
          ((int16_t)(int8_t)response[slot] << 8) | (uint16_t)response[slot + 1];

      counter++;
      slot = slot + 2;
    }
  } else {
    return error;
  }

  return 0;
}

int Sunlight::write_multiple_registers(uint8_t comAddr, uint16_t regAddr,
                                       uint16_t numReg, uint16_t writeVal[]) {
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

  // uint8_t writeValHi;
  // uint8_t writeValLo;

  int requestSize = 7 + numBytes;

  /* Assign the first 7 bytes to the request array */
  request[0] = comAddr;
  request[1] = funCode;
  request[2] = regAddrHi;
  request[3] = regAddrLo;
  request[4] = numRegHi;
  request[5] = numRegLo;
  request[6] = numBytes;

  /* Convert the words to be written into 2 bytes and assign them to the request
   * array */
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

  if (read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_MODE, numReg) !=
      0) {
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

  if (read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_MODE, numReg) !=
      0) {
    ESP_LOGE(TAG, "Failed to read Measurement Mode");
    return false;
  }

  if (values[0] != change[0]) {
    if (mode == CONTINUOUS) {
      ESP_LOGI(TAG, "Changing Measurement Mode to Continuous...");
    } else {
      ESP_LOGI(TAG, "Changing Measurement Mode to Single...");
    }

    if (write_multiple_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_MODE, numReg,
                                 change) != 0) {
      ESP_LOGE(TAG, "Failed to change measurement mode");
      return false;
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

  int error =
      read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_PERIOD, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read measurement period (%d)", error);
    ESP_LOGE(TAG, "Failed to change Measurement period");
    return false;
  }

  if (values[0] != seconds) {
    ESP_LOGI(TAG, "Changing measurement period to %ds", seconds);
    if (write_multiple_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_PERIOD, numReg,
                                 change) != 0) {
      ESP_LOGE(TAG, "Failed to change measurement period");
      return false;
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

  int error =
      read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_SAMPLES, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read measurement samples (%d)", error);
    ESP_LOGE(TAG, "Failed to change measurement samples");
    return false;
  }

  ESP_LOGI(TAG, "Changing measurement samples to %d", number);
  if (write_multiple_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_SAMPLES, numReg,
                               change) != 0) {
    ESP_LOGE(TAG, "Failed to change Measurement samples");
    return false;
  }

  ESP_LOGI(TAG, "Sensor restart is required to apply changes");
  return true;
}

int16_t Sunlight::read_sensor_measurements() {
  /* Function variables */
  int error;
  uint16_t numReg = 0x0004;
  uint16_t co2Value = 0;

  /* Read values */
  if ((error = read_input_registers(CO2_SUNLIGHT_ADDR, ERROR_STATUS, numReg)) !=
      0) {
    ESP_LOGE(TAG, "Failed to read input register (%d)", error);
  } else {
    /* Read CO2 concentration */
    co2Value = (int16_t)values[3];

    /* Read error status */
    ESP_LOGD(TAG, "Error Status: (0x%04x)", values[0]);
  }

  return co2Value;
}

int Sunlight::trigger_single_measurement() {
  /* HR34 address = 0x0021 (according to spec: "Start Single Measurement") */
  uint16_t payload[1];
  int err;

  payload[0] = 0x0001u; /* Write value 1 to start measurement */

  err = write_multiple_registers(CO2_SUNLIGHT_ADDR, HR34_START_SINGLE_ADDR, 1u,
                                 payload);
  if (err != 0) {
    ESP_LOGE(TAG, "Failed to trigger single measurement (error: %d)", err);
    return err; /* < 0 indicates error from Modbus layer */
  }

  ESP_LOGD(TAG, "Single measurement triggered successfully");
  return 0;
}

bool Sunlight::read_sensor_id() {
  /* Vendor Name */
  if (read_device_id(CO2_SUNLIGHT_ADDR, 0) != 0) {
    ESP_LOGE(TAG, "Failed to read vendor name");
    return false;
  }

  ESP_LOGI(TAG, "Vendor name: %s", device);

  /* ProductCode */
  if (read_device_id(CO2_SUNLIGHT_ADDR, 1) != 0) {
    ESP_LOGE(TAG, "Failed to read product code");
    return false;
  }

  ESP_LOGI(TAG, "Product code: %s", device);

  /* MajorMinorRevision */
  if (read_device_id(CO2_SUNLIGHT_ADDR, 2) != 0) {
    ESP_LOGE(TAG, "Failed to read MajorMinorRevision");
    return false;
  }

  ESP_LOGI(TAG, "MajorMinorRevision: %s", device);
  return true;
}

int Sunlight::startManualBackgroundCalibration() {
  /* Make sure reading sensor measurement is OK before start calibration*/
  read_sensor_measurements();
  if (values[0] & IR1_NO_ERROR) {
    ESP_LOGE(TAG,
             "Read sensor measurements failed, cannot start calibration (%.2x)",
             values[0]);
    return values[0];
  }

  uint16_t hr1_reset_value[] = {
      HR1_RESET_VALUE,
  };
  uint16_t hr2_background_cal[] = {
      HR2_BACKGROUND_CALIBRATION,
  };

  ESP_LOGI(
      TAG,
      "Reset last calibration result, before starting new calibration process");
  int error =
      write_multiple_registers(CO2_SUNLIGHT_ADDR, HR1, 1, hr1_reset_value);
  if (error != 0) {
    ESP_LOGE(TAG, "Failed reset last calibration result (%d)", error);
    return error;
  }

  uint32_t startTime = MILLIS();
  ESP_LOGI(TAG, "Start calibration...");
  error =
      write_multiple_registers(CO2_SUNLIGHT_ADDR, HR2, 1, hr2_background_cal);
  if (error != 0) {
    ESP_LOGE(TAG, "Failed start calibration (%d)", error);
    return error;
  }

  ESP_LOGI(TAG, "Calibration started, wait for calibration to complete");
  int attempts = 0;
  do {
    /* The calibration should be finished after next measurement period,
      so to simplicity the process - we just waiting for one (or two if we
      have a synchronization issue) measurement periods...
    */
    ESP_LOGI(TAG, "Wait for calibration status complete... [%d]", attempts + 1);
    
    // Check if sensor is in single mode before triggering
    if (this->is_single_mode()) {
      int triggerResult = this->trigger_single_measurement();
      if (triggerResult == 0) {
        // Wait for measurement to complete
        vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3 seconds
        ESP_LOGD(TAG, "Single measurement triggered for calibration reading...");
      } else {
        ESP_LOGW(TAG, "Failed to trigger single measurement in calibration, trying to "
                      "read anyway...");
      }
    } else {
      ESP_LOGD(TAG, "Sensor in continuous mode, no trigger needed");
    }
    
    vTaskDelay(pdMS_TO_TICKS(CALIBRATION_STATUS_CHECK_INTERVAL));
    /* First check ErrorStatus, probably the calibration is fail due to
     * non-stable environment */
    if ((error = read_input_registers(CO2_SUNLIGHT_ADDR, ERROR_STATUS, 1)) ==
        0) {
      if (values[0] & IR1_CALIBRATION_ERROR) {
        ESP_LOGE(TAG, "Sensor return calibration error (SLAVE_FAILURE)");
        error = SLAVE_FAILURE;
      }
      /* Second, check Calibration status */
      else if ((error = read_holding_registers(CO2_SUNLIGHT_ADDR, HR1, 1)) ==
               0) {
        /* Is calibration completed? */
        if (values[0] & HR1_BACKGROUND_CALIBRATION) {
          uint32_t finish = MILLIS() - startTime;
          ESP_LOGI(TAG, "Calibration completed in %lu ms", finish);
          break;
        }
        /* Check timeout, probably we used wrong measurement period to wait
           for calibration complete */
        else if (++attempts == CALIBRATION_WAIT_COMPLETE_COUNTER) {
          ESP_LOGW(TAG, "Timeout wait for calibration to complete, might take "
                        "longer to complete. Make sure "
                        "before retry!");
          error = SLAVE_FAILURE;
        }
      }
    }
  } while (error == 0);

  return error;
}

bool Sunlight::isABCEnabled() {
  /* Function variables */
  uint16_t numReg = 0x0001;

  int error = read_holding_registers(CO2_SUNLIGHT_ADDR, METER_CONTROL, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read meter control (%d)", error);
    return false;
  }

  return (bitRead(values[0], 1) == 0);
}

bool Sunlight::setMeterControlBit(uint8_t target, bool newValue, uint8_t bit) {
  /* Function variables */
  uint16_t numReg = 0x0001;

  int error = read_holding_registers(target, METER_CONTROL, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read meter control (%d)", error);
    return false;
  }
  uint16_t meterControlVal = values[0];
  bool isHigh = bitRead(meterControlVal, bit) == 0;

  if (isHigh == newValue) {
    ESP_LOGD(TAG, "Meter control already in desired status");
    return true;
  }

  if (newValue) {
    // Enabling
    bitClear(meterControlVal, bit);
  } else {
    // Disabling
    bitSet(meterControlVal, bit);
  }

  uint16_t change[] = {meterControlVal};
  if (write_multiple_registers(target, METER_CONTROL, numReg, change) != 0) {
    ESP_LOGE(TAG, "Failed to change meter control status");
    return false;
  }

  return true;
}

void Sunlight::setABC(bool enable) {
  if (!this->setMeterControlBit(CO2_SUNLIGHT_ADDR, enable, 1)) {
    ESP_LOGE(TAG, "Failed to set ABC period");
  }
  ESP_LOGI(TAG, "ABC period already in desired value");
}

bool Sunlight::setABCPeriod(uint16_t hours) {
  const uint16_t registerAddr = ABC_PERIOD;
  const uint16_t numReg = 1;

  // Read current value to avoid unnecessary writes
  int error = read_holding_registers(CO2_SUNLIGHT_ADDR, registerAddr, numReg);
  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read ABC period register (%d)", error);
    return false;
  }

  if (values[0] == hours) {
    ESP_LOGI(TAG, "ABC period already set to desired value");
    return true;
  }

  uint16_t newValue[] = {hours};
  error = write_multiple_registers(CO2_SUNLIGHT_ADDR, registerAddr, numReg,
                                   newValue);
  if (error != 0) {
    ESP_LOGE(TAG, "Failed to write ABC period (%d)", error);
    return false;
  }

  ESP_LOGD(TAG, "Success set ABC period");
  return true;
}

void Sunlight::setNRDY(bool enable) {
  if (!this->setMeterControlBit(CO2_SUNLIGHT_ADDR, enable, 0)) {
    DBG("nRDY value change not needed");
  }
}

bool Sunlight::is_single_mode() {
  /* Function variables */
  uint16_t numReg = 0x0001;

  int error = read_holding_registers(CO2_SUNLIGHT_ADDR, MEASUREMENT_MODE, numReg);

  if (error != 0) {
    ESP_LOGE(TAG, "Failed to read measurement mode (%d)", error);
    return false; // Assume continuous mode on error for safety
  }

  return (values[0] == SINGLE);
}
