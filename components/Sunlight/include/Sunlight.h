#ifndef SUNLIGHT_H
#define SUNLIGHT_H

#include <stdint.h>
#include "AirgradientSerial.h"

class Sunlight {

  /*
   * The delay when waiting for responses, in milliseconds.
   * Length based on the documentation, "Modbus on Senseair
   * Sunlight".
   */
  static const int WAIT_MS = 180;
  /* For baudrate equal 9600 the Modbus 3.5T interval is close to 3.5 ms, we round it to 4 ms*/
  static const int INTER_PACKET_INTERVAL_MS = 5;

  /* Wait interval between manual calibration attempt */
  static const int CALIBRATION_WAIT_INTERVAL = 4000;

  /* Error codes */
  static const int COMMUNICATION_ERROR = -1;
  static const int ILLEGAL_FUNCTION = 1;
  static const int ILLEGAL_DATA_ADDRESS = 2;
  static const int ILLEGAL_DATA_VALUE = 3;
  static const int SLAVE_FAILURE = -4;

  /* Error statuses */
  static const uint16_t IR1_FATAL_ERROR = 0x0001u;
  static const uint16_t IR1_I2C_ERROR = 0x0002u;
  static const uint16_t IR1_ALGORITHM_ERROR = 0x0004u;
  static const uint16_t IR1_CALIBRATION_ERROR = 0x0008u;
  static const uint16_t IR1_SELFDIAG_ERROR = 0x0010u;
  static const uint16_t IR1_OUT_OF_RANGE_ERROR = 0x0020u;
  static const uint16_t IR1_MEMORY_ERROR = 0x0040u;
  static const uint16_t IR1_NO_MEASUREMENT_ERROR = 0x0080u;
  static const uint16_t IR1_NO_ERROR = 0x0000u;

  /* Sensors registers */
  static const uint16_t HR1 = 0u;
  static const uint16_t HR2 = 1u;

  /* Calibration statuses fro HR1 register */
  static const uint16_t HR1_RESET_VALUE = 0x0000u;
  static const uint16_t HR1_BACKGROUND_CALIBRATION = 0x0020u;

  /* Calibration command for HR2 register */
  static const uint16_t HR2_RESTORE_FACTORY_CALIBRATION = 0x7C02u;
  static const uint16_t HR2_BACKGROUND_CALIBRATION = 0x7C06u;

  /* Function codes */

  /* Register addresses */
  static const uint16_t ERROR_STATUS = 0x0000;
  static const uint16_t MEASUREMENT_MODE = 0x000A;
  static const uint16_t MEASUREMENT_PERIOD = 0x000B;
  static const uint16_t MEASUREMENT_SAMPLES = 0x000C;
  static const uint16_t METER_CONTROL = 0x0012;

  /* Measurement modes */
  static const uint16_t CONTINUOUS = 0x0000;
  static const uint16_t SINGLE = 0x0001;

  /**
   * Arrays for request, responses and register values
   * Sizes based on:
   * MODBUS APPLICATION PROTOCOL SPECIFICATION V1.1b3
   * Available on the www.modbus.org website
   */
  /* Array for sending requests */
  uint8_t request[256];
  /* Array for receiving responses */
  uint8_t response[256];
  /* Array for storing register values from responses */
  uint16_t values[256];
  /* Array for storing strings from reading objects */
  char device[14];

public:
  Sunlight(AirgradientSerial &serial);
  ~Sunlight() {};

  /**
   * @brief  Reads and prints the sensor's current measurement mode,
   *         measurement period and number of samples.
   *
   * @note   This example shows a simple way to read the sensor's
   *         measurement configurations.
   * @retval None
   */
  void read_sensor_config();

  /**
   * @brief  Reads and prints the sensor's current CO2 value and
   *         error status.
   *
   * @note   This example shows a simple way to read the sensor's
   *         CO2 measurement and error status.
   * @retval co2
   */
  int16_t read_sensor_measurements();

  /**
   * @brief  Reads and prints the sensor's device identification.
   *
   * @note   This example shows a simple way to read and print the
   *         sensor's Vendor Name, ProductCode and MajorMinorRevision.
   * @retval None
   */
  void read_sensor_id();

  /**
   * @brief  Changes the sensor's current measurement mode
   *
   * @param  mode:
   * @note   This example shows a simple way to change the sensor's
   *         measurement mode. The sensor has to be manually restarted after the
   *         changes.
   * @retval true, device restart required
   */
  bool set_measurement_mode(uint8_t mode);

  /**
   * @retval true, device restart required
   */
  bool set_measurement_period(uint16_t seconds);

  /**
   * @retval true, device restart required
   */
  bool set_measurement_samples(uint16_t number);

  /**
   * @brief  Make background calibration and report
   *         error status.
   */
  int startManualBackgroundCalibration();

  /**
   * @retval true, ABC calibration is enabled on device
   */
  bool isABCEnabled();

  void setABC(bool enable);

  void setNRDY(bool enable);

private:
  const char *const TAG = "Sunlight";
  AirgradientSerial &_agSerial;
  /**
   * @brief  Reads one of the device's ID objects.
   *
   * @param  comAddr:      Communication address
   *         objId:        The type of object that is to be read
   * @retval None
   */
  int read_device_id(uint8_t comAddr, uint8_t objId);

  /**
   * @brief  Reads slave response.
   *
   * @param  waitBytes:    Number of expected bytes for receive packet
   *         funCode:      Function code
   * @note   This function stores the values read through a global
   *         array, which can then be read to obtain the values.
   * @retval Error status, >0 on success (response size), -1 on communication error
   *         or time-out, and 1 - 3 for exceptions.
   */
  int modbus_read_response(int waitBytes, uint8_t funCode);

  /**
   * @brief  Reads multiple holding registers.
   *
   * @param  comAddr:      Communication address
   *         regAddr:      Starting register address
   *         numReg:       Number of registers to read from
   * @note   This function stores the values read through a global
   *         array, which can then be read to obtain the values.
   * @retval Error status, 0 on success, -1 on communication error
   *         or time-out, and 1 - 3 for exceptions.
   */
  int read_holding_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg);

  /**
   * @brief  Reads multiple input registers.
   *
   * @param  comAddr:      Communication address
   *         regAddr:      Starting register address
   *         numReg:       Number of registers to read from
   * @note   This function stores the values read through a global
   *         array, which can then be read to obtain the values.
   * @retval Error status, 0 on success, -1 on communication error
   *         or time-out, and 1 - 3 for exceptions.
   */
  int read_input_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg);

  /**
   * @brief  Computes a Modbus RTU message CRC, for a given message.
   *
   * @param  pdu[]: An array containing the message
   *         len: the length of the array
   * @note   The bytes in the return value needs to be switched for
   *         them to be in the right order in the message.
   * @retval The CRC for the message
   */
  uint16_t _generate_crc(uint8_t pdu[], int len);

  /**
   * @brief  A handler for possible exceptions and errors.
   *
   * @param  pdu[]: An array containing the response from a request
   *         funCode: The function code used for the request
   *         len: The length of the pdu
   * @retval  -1 for incorrect CRC, 0 on success, 1 for Illegal
   *          Function, 2 for Illegal Data Address, and 3 for
   *          Illegal Data Value
   */
  int _handler(uint8_t pdu[], uint8_t funCode, int len);

  /**
   * @brief  Writes to multiple holding registers.
   *
   * @param  comAddr:  Communication address
   *         regAddr:  Register address
   *         numReg:   Number of registers to write to
   *         writeVal: The values to write to the registers
   * @retval Error status, 0 on success, -1 on communication error
   *         or time-out, and 1 - 3 for exceptions.
   */
  int write_multiple_registers(uint8_t comAddr, uint16_t regAddr, uint16_t numReg,
                               uint16_t writeVal[]);

  bool setMeterControlBit(uint8_t target, bool enable, uint8_t bit);
};

#endif // !SUNLIGHT_H
