/*!
 * @file DFRobot_IICSerial.h
 * @brief Define the basic structure of class DFRobot_IICSerial
 * @n This is a library for IIC to UART module, the maximum rate is 1Mbps;
 * @n The band rate, word length, and check format of every sub UART can be set independently;
 * @n The module can provide at most 2Mbps communication rate;
 * @n Each sub UART is able to receive/transmit independent 256 bytes FIFO hardware cache;
 * @n Users can configure FIFO interrupt by programming.
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-07-28
 * @https://github.com/DFRobot/DFRobot_IICSerial
 */
#ifndef AIRGRADIENT_IIC_SERIAL_H
#define AIRGRADIENT_IIC_SERIAL_H

// #include <Arduino.h>
#include <inttypes.h>
#include <stddef.h>
// #include "Stream.h"

#include "AirgradientSerial.h"
#include "driver/i2c_master.h"

// #if ARDUINO >= 100
// #include "Arduino.h"
// #else
// #include "WProgram.h"
// #endif
// #include <Wire.h>
//

// #if 0  //< Change 0 to 1 to open debug macro and check program debug information
// #define DBG(...) {Serial.print("["); Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
// #else
// #define DBG(...)
// #endif

#if !defined(SERIAL_RX_BUFFER_SIZE)
#if ((RAMEND - RAMSTART) < 1023)
#define SERIAL_RX_BUFFER_SIZE 16
#else
#define SERIAL_RX_BUFFER_SIZE 32
#endif
#endif
#if (SERIAL_RX_BUFFER_SIZE > 256)
typedef uint16_t rx_buffer_index_t;
#else
typedef uint8_t rx_buffer_index_t;
#endif

class AirgradientIICSerial : public AirgradientSerial {
public:
#define MAX_WK2132_ADDR 8
/**
 * @brief Data format: N for no parity, Z for 0 parity, O for Odd parity, E for Even parity, F for 1 parity.
 * @n 8 represents the number of data bit, 1 or 2 for the number of stop bit.
 */
#define IICSerial_8N1 0x00
#define IICSerial_8N2 0x01
#define IICSerial_8Z1 0x08
#define IICSerial_8Z2 0x09
#define IICSerial_8O1 0x0A
#define IICSerial_8O2 0x0B
#define IICSerial_8E1 0x0C
#define IICSerial_8E2 0x0D
#define IICSerial_8F1 0x0E
#define IICSerial_8F2 0x0F
#define SUBUART_CHANNEL_1 0x00   //< Sub UART channel1
#define SUBUART_CHANNEL_2 0x01   //< Sub UART channel2
#define SUBUART_CHANNEL_ALL 0x11 //< All sub channels
#define IIC_ADDR_FIXED                                                                             \
  0x10 //< The 4th and 3rd bits of IIC address are fixed, value 1 and 0 respectively

#define ERR_OK 0
#define ERR_REGDATA -1
#define ERR_READ -2
#define FOSC 14745600L       //< External cystal frequency 14.7456MHz
#define OBJECT_REGISTER 0x00 //< Register object
#define OBJECT_FIFO 0x01     //< FIFO buffer object
#ifdef ARDUINO_ARCH_NRF5
#define IIC_BUFFER_SIZE 63 //< micro:bit IIC can transmit at most 63 bytes each time
#elif ARDUINO_ARCH_MPYTHON
#define IIC_BUFFER_SIZE 31 //< mPython IIC can transmit at most 31 bytes each time
#else
#define IIC_BUFFER_SIZE                                                                            \
  32 //< UNO, Mega2560, Leonardo(AVR series), IIC can transmit at most 32 bytes each time
#endif

  typedef enum {
    eNormalMode = 0,
    //eIrDAMode
  } eCommunicationMode_t;

  typedef enum {
    eNormal = 0,
    //eLineBreak
  } eLineBreakOutput_t;

protected:
  /**
   * @struct sIICAddr_t
   * @brief WK2132 IIC address format:
   * @n -------------------------------------------------------------------------
   * @n |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
   * @n -------------------------------------------------------------------------
   * @n |   0    |   A1   |   A0   |   1    |   0    |   C1   |   C0   |   0/1  |
   * @n -------------------------------------------------------------------------
   * @n |                    addrPre                 |       uart      |   type |
   * @n -------------------------------------------------------------------------
   */
  typedef struct {
    uint8_t type : 1; /**< The 0 bit of IIC address, 0 for register, 1 for FIFO */
    uint8_t
        uart : 2; /**< The 2nd and 1st bits of IIC address, sub UART channel 1:0x00, sub UART channel 2:0x01 */
    uint8_t
        addrPre : 5; /**< The 3rd to 6th bits of IIC address, the 5th and 6th bits can be selected by the external DIP switch A1,A0. */
  } __attribute__((packed)) sIICAddr_t;

  /**
   * @struct sSierReg_t
   * @brief SIER description of WK2132 sub UART interrupt enable register:
   * @n --------------------------------------------------------------------------------------------
   * @n |    b7    |   b6   |   b5   |   b4   |      b3     |     b2     |     b1     |     b0     |
   * @n --------------------------------------------------------------------------------------------
   * @n | FERR_IEN |          RSV             | TFEMPTY_IEN | TFTRIG_IEN | RXOVT_IEN  | RFTRIG_IEN |
   * @n --------------------------------------------------------------------------------------------
   */
  typedef struct {
    uint8_t rFTrig : 1;  /**< Receive FIFO contact interrupt enable bit, 1-enable, 0-disable */
    uint8_t rxOvt : 1;   /**< Receive FIFO timeout interrupt enable bit, 1-enable, 0-disable */
    uint8_t tfTrig : 1;  /**< Transmit FIFO contact interrupt enable bit, 1-enable, 0-disable */
    uint8_t tFEmpty : 1; /**< Transmit FIFO null interrupt enable bit, 1-enable, 0-disable */
    uint8_t rsv : 3;     /**< Reserved bit */
    uint8_t fErr : 1;    /**< Receive FIFO data error interrupt enable bit, 1-enable, 0-disable */
  } __attribute__((packed)) sSierReg_t;

  /**
   * @struct sFcrReg_t
   * @brief FCR description of WK2132 sub UART FIFO control register:
   * @n -------------------------------------------------------------------------
   * @n |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
   * @n -------------------------------------------------------------------------
   * @n |      TFTRIG     |      RFTRIG     |  TFEN  |  RFEN  |  TFRST |  RFRST |
   * @n -------------------------------------------------------------------------
   */
  typedef struct {
    uint8_t
        rfRst : 1; /**< Sub UART receive FIFO reset bit, 1-reset FIFO, 0-disable reset, write 1 to reset, auto back to 0 when completed */
    uint8_t
        tfRst : 1; /**< Sub UART transmit FIFO reset bit, 1-reset FIFO, 0-disable reset, write 1 to reset, auto back to 0 when completed */
    uint8_t rfEn : 1; /**< Sub UART receive FIFO enable bit, 1-enable, 0-disable */
    uint8_t tfEn : 1; /**< Sub UART transmit FIFO enable bit, 1-enable, 0-disable */
    uint8_t
        rfTrig : 2; /**< Sub UART receive FIFO contact setting bit, 00-8Byte(default), 01-16Byte, 10-24Byte, 11-18Byte */
    uint8_t
        tfTrig : 2; /**< Sub UART transmit FIFO contact setting bit, 00-8Byte(default), 01-16Byte, 10-24Byte, 11-18Byte */
  } __attribute__((packed)) sFcrReg_t;

  /**
   * @struct sScrReg_t
   * @brief SCR description of WK2132 sub UART control register:
   * @n -------------------------------------------------------------------------
   * @n |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
   * @n -------------------------------------------------------------------------
   * @n |                     RSV                    | SLEEPEN|  TXEN  |  RXEN  |
   * @n -------------------------------------------------------------------------
   */
  typedef struct {
    uint8_t rxEn : 1;    /**< Sub UART receive enable bit, 1-enable, 0-disable */
    uint8_t txEn : 1;    /**< Sub UART transmit enable bit, 1-enable, 0-disable */
    uint8_t sleepEn : 1; /**< Sub UART sleep enable bit, 1-enable, 0-disable */
    uint8_t rsv : 5;     /**< reserved bit */
  } __attribute__((packed)) sScrReg_t;

  /**
   * @struct sLcrReg_t
   * @brief LCR description of WK2132 sub UART configuration register:
   * @n -------------------------------------------------------------------------
   * @n |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
   * @n -------------------------------------------------------------------------
   * @n |        RSV      |  BREAK |  IREN  |  PAEN  |      PAM        |  STPL  |
   * @n -------------------------------------------------------------------------
   */
  typedef struct {
    uint8_t
        format : 4; /**< Sub UART data format: PAEN sub UART check enable bit, 1-with parity bit(9-bits data), 0-no parity bit (8-bits data) PAM sub UART check mode selection bit, take effect when PAEN bit is 1, 00-0 parity(default), 01-Odd parity 10-Even parity, 11-1 parity, STPL sub UART stop bit length, 0-1bit, 1-2bits, etc.*/
    uint8_t irEn : 1; /**< Sub UART IR enable bit, 0-normal mode, 1-IR mode */
    uint8_t
        lBreak : 1; /**< Sub UART Line-Break output control bit, 0-output normally, 1-Line-Break output (TX force output 0) */
    uint8_t rsv : 2; /**< Reserved bit */
  } __attribute__((packed)) sLcrReg_t;

  /*
   FSR description of WK2132 sub UART FIFO state register:
     * -------------------------------------------------------------------------
     * |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
     * -------------------------------------------------------------------------
     * |  RFOE  |  RFBI  |  RFFE  |  RFPE  |  RDAT  |  TDAT  |  TFULL |  TBUSY |
     * -------------------------------------------------------------------------
  */
  /**
   * @struct sFsrReg_t
   * @brief FSR description of WK2132 sub UART FIFO state register:
   * @n -------------------------------------------------------------------------
   * @n |   b7   |   b6   |   b5   |   b4   |   b3   |   b2   |   b1   |   b0   |
   * @n -------------------------------------------------------------------------
   * @n |  RFOE  |  RFBI  |  RFFE  |  RFPE  |  RDAT  |  TDAT  |  TFULL |  TBUSY |
   * @n -------------------------------------------------------------------------
   */
  typedef struct {
    uint8_t
        tBusy : 1; /**< Sub UART transmit TX busy flag bit, 0-sub UART transmit TX null, 1-sub UART transmit TX busy */
    uint8_t
        tFull : 1; /**< Sub UART transmit TX full flag bit, 0-sub UART transmit FIFO not full, 1-sub UART transmit FIFO full */
    uint8_t
        tDat : 1; /**< Sub UART transmit FIFO null flag, 0-sub UART transmit FIFO null, 1-sub UART transmit FIFO not null */
    uint8_t
        rDat : 1; /**< Sub UART receive FIFO null flag, 0-sub UART receive FIFO null, 1-sub UART receive FIFO not null */
    uint8_t
        rFpe : 1; /**< Sub UART receive FIFO data check error flag bit, 0- no PE error, 1-PE error */
    uint8_t
        rFfe : 1; /**< Sub UART receive FIFO data frame error flag bit, 0-no FE error, 1-FE error */
    uint8_t
        rFbi : 1; /**< Sub UART receive FIFO data Line-break error, 0-no Line-Break error, 1-Line-Break error */
    uint8_t
        rFoe : 1; /**< Sub UART receive FIFO data overflow error flag bit, 0-no OE error, 1-OE error */
  } __attribute__((packed)) sFsrReg_t;

  typedef enum {
    clock = 0, /**< Operate global control register, control sub UART clock */
    rst, /**< Operate global sub UART reset register, reset a sub UART independently through software */
    intrpt /**< Operate global interrupt register, control sub UART total interrupt */
  } eGlobalRegType_t;

  typedef enum { page0 = 0, page1, pageTotal } ePageNumber_t;

public:
  /**
   * @fn DFRobot_IICSerial
   * @brief Constructor
   * @param wire I2C bus pointer object, default Wire
   * @param subUartChannel sub UART channel, WK2132 has two sub UARTs: SUBUART_CHANNEL_1 or SUBUART_CHANNEL_2
   * @param IA1 corresponds with IA1 Level(0 or 1) of DIP switch on the module, and is used for configuring
   * @n the IIC address of the 6th bit value(default: 1).
   * @param IA0 corresponds with IA0 Level(0 or 1) of DIP switch on the module, and is used for configuring
   * @n IIC address of the 5th bit value(default: 1).
   * @n IIC address configuration:
   * @n 7   6   5   4   3   2   1   0
   * @n 0  IA1 IA0  1   0  C1  C0  0/1
   * @n IIC address only has 7 bits, while there are 8 bits for one byte, so the extra one bit will be filled as 0.
   * @n The 6th bit corresponds with IA1 Level of DIP switch, can be configured manually.
   * @n The 5th bit corresponds with IA0 Level of DIP switch, can be configured manually.
   * @n The 4th and 3rd bits are fixed, value 1 and 0 respectively.
   * @n The values of the 2nd and 1st bits are the sub UART channels, 00 for sub UART 1, 01 for sub UART 2.
   * @n The 0 bit represents the operation object: 0 for register, 1 for FIFO cache.
   */
  // AirgradientIICSerial(TwoWire &wire = Wire, uint8_t subUartChannel = SUBUART_CHANNEL_1, uint8_t IA1 = 1, uint8_t IA0 = 1);
  AirgradientIICSerial(i2c_master_bus_handle_t i2c_bus_handle,
                       uint8_t subUartChannel = SUBUART_CHANNEL_1, uint8_t IA1 = 1,
                       uint8_t IA0 = 1);
  ~AirgradientIICSerial();

  /**
   * @fn begin(long unsigned baud)
   * @brief Init function, set band rate of sub UART
   * @param baud Band rate
   * @n Besides the commonly-used 9600, 57600, 115200, it also supports:
   * @n 2400, 4800, 7200, 14400, 19200, 28800, 38400, 76800, 153600, 230400, 460800, 307200, 921600
   * @return Return 0 if it succeeds, otherwise return non-zero
   */
  int begin(long unsigned baud) { return begin(baud, IICSerial_8N1, eNormalMode, eNormal); }

  /**
   * @fn begin(long unsigned baud, uint8_t format)
   * @brief Init function, set sub UART band rate, data format
   * @param baud Band rate
   * @param format Data format: IICSerial_8N1, IICSerial_8N2, IICSerial_8Z1
   * @n IICSerial_8Z2, IICSerial_8O1, IICSerial_8O2, IICSerial_8E1, IICSerial_8E2
   * @n IICSerial_8F1, IICSerial_8F2
   * @return Return 0 if it succeeds, otherwise return non-zero
   */
  int begin(long unsigned baud, uint8_t format) {
    return begin(baud, format, eNormalMode, eNormal);
  }

  /**
   * @fn end
   * @brief Release sub UART to clean up all registers in Sub UART. Call function begin() again to make it work.
   */
  void end();

  /**
   * @fn available
   * @brief Get the number of bytes in receive buffer, it should be the total number of bytes in FIFO
   * @n receive buffer(256B) and self-defined _rx_buffer(31B).
   * @return Return the number of bytes in receive buffer
   */
  int available(void);

  /**
   * @fn peek
   * @brief Return the data of 1 byte without deleting the data in the receive buffer
   * @return Return the readings
   */
  int peek(void);

  /**
   * @fn read(void)
   * @brief Read 1 byte in receive buffer, this operation will delete the data in the buffer.
   * @return Return the readings
   */
  int read(void);

  /**
   * @fn read(void *pBuf, size_t size)
   * @brief Read a specified number of character from FIFO and store them into a array.
   * @n This operation doesn't involve receive buffer.
   * @param pBuf Array for storing data
   * @param size The number of character to be read
   * @return Return the readings
   */
  size_t read(void *pBuf, size_t size);

  /**
   * @fn flush
   * @brief Wait for the data to be transmited completely
   */
  void flush(void);

  void print(const char *str);

  /**
   * @fn write
   * @brief Write data into transmit FIFO cache
   * @param pBuf Store buffer for the data to be read
   * @param size Length of the data to be read
   * @return Output the number of bytes
   */
  int write(const uint8_t *pBuf, int size);
  // using Print::write; /*!< pull in write(str) and write(buf, size) from Print */
  operator bool() { return true; }

  bool isChannelInSleep(uint8_t subUartChannel);
  void printAllRegsForCurrentCh();
  void prepareSleep();
  void turnOffClock();

protected:
  /**
   * @fn begin(long unsigned baud, uint8_t format, eCommunicationMode_t mode, eLineBreakOutput_t opt)
   * @brief Init function, set the band rate of sub UART, data format, communication mode, and Line-Break output
   * @param baud Band rate
   * @param format Sub UART data format: IICSerial_8N1, IICSerial_8N2, IICSerial_8Z1
   * @n IICSerial_8Z2, IICSerial_8O1, IICSerial_8O2, IICSerial_8E1, IICSerial_8E2
   * @n IICSerial_8F1, IICSerial_8F2
   * @param mode Sub UART communciation mode, can set to UART mode, all enumeration values in eCommunicationMode_t
   * @param opt Sub UART Line-Break output control bit, can set to normal output (0) and Line-Break output (1),
   * @n all enumeration values in eLineBreakOutput_t or 0/1
   * @return Return 0 if init succeeds, otherwise return non-zero
   */
  int begin(long unsigned baud, uint8_t format, eCommunicationMode_t mode, eLineBreakOutput_t opt);

  /**
   * @fn subSerialConfig
   * @brief Sub UART parameter configuration
   * @param subUartChannel Sub UART channel: SUBUART_CHANNEL_1 or SUBUART_CHANNEL_2
   */
  void subSerialConfig(uint8_t subUartChannel);

  /**
   * @fn subSerialGlobalRegEnable
   * @brief Sub UART global register enable
   * @param subUartChannel Sub UART channel: SUBUART_CHANNEL_1 or SUBUART_CHANNEL_2
   * @param type Global register type, all enumeration values in eGlobalRegType_t
   */
  void subSerialGlobalRegEnable(uint8_t subUartChannel, eGlobalRegType_t type);

  /**
   * @fn subSerialRegConfig
   * @brief Sub UART register configuration: SIER, FCR, LCR
   * @param reg Register address
   * @param pValue Data store buffer, 1 byte
   */
  void subSerialRegConfig(uint8_t reg, void *pValue);

  /**
   * @fn getGlobalRegType
   * @brief Get global register address
   * @param type Global register type, all enumeration values in eGlobalRegType_t
   * @return Return register address
   */
  uint8_t getGlobalRegType(eGlobalRegType_t type);

  /**
   * @fn setSubSerialBaudRate
   * @brief Set sub UART band rate
   * @param baud Band rate
   */
  void setSubSerialBaudRate(unsigned long baud);

  /**
   * @fn setSubSerialConfigReg
   * @brief Set sub UART configuration register
   * @param format Sub UART data format: IICSerial_8N1, IICSerial_8N2, IICSerial_8Z1
   * @n IICSerial_8Z2, IICSerial_8O1, IICSerial_8O2, IICSerial_8E1, IICSerial_8E2
   * @n IICSerial_8F1, IICSerial_8F2
   * @param mode Sub UART communicaiton mode, can be set to IR mode or normal mode, all enumeration values in eCommunicationMode_t
   * @param opt Sub UART Line-Break output control bit, can be set to normal output or Line-Break output, all enumeration values in eLineBreakOutput_t
   */
  void setSubSerialConfigReg(uint8_t format, eCommunicationMode_t mode, eLineBreakOutput_t opt);

  /**
   * @fn subSerialPageSwitch
   * @brief Sub UART register page switch
   * @param page Page number, all enumeration values in ePageNumber_t
   */
  void subSerialPageSwitch(ePageNumber_t page);

  /**
   * @fn updateAddr
   * @brief Update module IIC address before starting communication
   * @param pre The first 5 bits of IIC address, transfered from constructor DFRobot_WK2132
   * @param subUartChannel Sub UART channel: SUBUART_CHANNEL_1 or SUBUART_CHANNEL_2
   * @param obj Object to be operated: register or FIFO, fill as OBJECT_REGISTER or OBJECT_FIFO
   * @return Return 8-bits IIC address
   */
  uint8_t updateAddr(uint8_t pre, uint8_t subUartChannel, uint8_t obj);

  /**
   * @fn readFIFOStateReg
   * @brief Read FIFO state register
   * @return Return the value of sFsrReg_t construct object
   */
  sFsrReg_t readFIFOStateReg();

  /**
   * @fn subSerialChnnlSwitch
   * @brief Sub UART channel switch
   * @param subUartChannel Sub UART channel: SUBUART_CHANNEL_1 or SUBUART_CHANNEL_2
   * @return Return the switched channel
   */
  uint8_t subSerialChnnlSwitch(uint8_t subUartChannel);

  /**
   * @fn sleep
   * @brief Sub UART enters sleep state
   */
  void sleep();

  /**
   * @fn wakeup
   * @brief Sub UART wakes up from sleep
   */
  void wakeup();

  /**
   * @fn writeReg
   * @brief Write register function
   * @param reg  Register address  8bits
   * @param pBuf Store buffer for the data to be written
   * @param size Length of the data to be written
   */
  void writeReg(uint8_t reg, const void *pBuf, size_t size);

  /**
   * @fn writeFIFO
   * @brief Write FIFO buffer
   * @param pBuf Store buffer for the data to be written
   * @param size Length of the data to be written
   */
  void writeFIFO(void *pBuf, size_t size);

  /**
   * @fn readReg
   * @brief Read register function
   * @param reg  Register address 8bits
   * @param pBuf Store buffer for the data to be read
   * @param size Length of the data to be read
   * @return Return the actual length, 0 means failed to read
   */
  uint8_t readReg(uint8_t reg, void *pBuf, size_t size);

  /**
   * @fn readFIFO
   * @brief Read FIFO buffer
   * @param pBuf Store buffer for the data to be read
   * @param size Length of the data to be read
   * @return Return the actual length, 0 means failed to read
   */
  uint8_t readFIFO(void *pBuf, size_t size);

  i2c_master_dev_handle_t getDeviceHandle(uint8_t addr);

protected:
  volatile rx_buffer_index_t _rx_buffer_head;
  volatile rx_buffer_index_t _rx_buffer_tail;
  unsigned char _rx_buffer[SERIAL_RX_BUFFER_SIZE];

private:
  // Define a structure to hold address and device handle
  typedef struct {
    uint8_t address;
    i2c_master_dev_handle_t dev_handle;
  } DeviceInfo;

  // Array to store devices
  DeviceInfo dev_handle_map[MAX_WK2132_ADDR] = {{0}};

  i2c_master_bus_handle_t _i2c_bus_handle;
  uint8_t _addr;
  uint8_t _subSerialChannel;
};

#endif // AIRGRADIENT_IIC_SERIAL_H
