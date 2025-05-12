/*!
 * Modified version
 *
 * @file DFRobot_IICSerial.h
 * @brief Define the basic structure of class DFRobot_IICSerial
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Arya](xue.peng@dfrobot.com)
 * @version  V1.0
 * @date  2019-07-28
 * @https://github.com/DFRobot/DFRobot_IICSerial
 */

// #define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

#include <cstdint>
#include <cstring>
#include "esp_log.h"

#include "DFRobot_IICSerial.h"
#include "freertos/FreeRTOS.h"

#define MILLIS() ((uint32_t)(esp_timer_get_time() / 1000))

/**
 * @brief Global register
 */
#define REG_WK2132_GENA 0x00 //< Global control register, control sub UART clock
#define REG_WK2132_GRST                                                                            \
  0x01 //< Global sub UART reset register, reset a sub UART independently through software
#define REG_WK2132_GMUT                                                                            \
  0x02 //< Global main UART control register, and will be used only when the main UART is selected as UART, no need to be set here.
#define REG_WK2132_GIER 0x10 //< Global interrupt register, control sub UART total interrupt.
#define REG_WK2132_GIFR                                                                            \
  0x11 //< Global interrupt flag register, only-read register: indicate if there is a interrupt occuring on a sub UART.

/**
 * @brief Page control register
 */
#define REG_WK2132_SPAGE 0x03 //< sub UART page control register

/**
 * @brief sub UART register SPAGE0
 */
#define REG_WK2132_SCR 0x04   //< Sub UART control register
#define REG_WK2132_LCR 0x05   //< Sub UART configuration register
#define REG_WK2132_FCR 0x06   //< Sub UART FIFO control register
#define REG_WK2132_SIER 0x07  //< Sub UART interrupt enable register
#define REG_WK2132_SIFR 0x08  //< Sub UART interrupt flag register
#define REG_WK2132_TFCNT 0x09 //< Sub UART transmit FIFO register, OR register
#define REG_WK2132_RFCNT 0x0A //< Sub UART transmit FIFO register, OR register
#define REG_WK2132_FSR 0x0B   //< Sub UART FIFO register, OR register
#define REG_WK2132_LSR 0x0C   //< Sub UART receive register, OR register
#define REG_WK2132_FDAT 0x0D  //< Sub UART FIFO data register
/**
 * @brief Sub UART register SPAGE1
 */
#define REG_WK2132_BAUD1 0x04 //< Sub UART band rate configuration register high byte
#define REG_WK2132_BAUD0 0x05 //< Sub UART band rate configuration register low byte
#define REG_WK2132_PRES 0x06  //< Sub UART band rate configuration register decimal part
#define REG_WK2132_RFTL 0x07  //< Sub UART receive FIFO interrupt trigger configuration register
#define REG_WK2132_TFTL 0x08  //< Sub UART transmit FIFO interrupt trigger configuration register

DFRobot_IICSerial::DFRobot_IICSerial(i2c_master_bus_handle_t i2c_bus_handle,
                                           uint8_t subUartChannel, uint8_t IA1, uint8_t IA0) {
  _i2c_bus_handle = i2c_bus_handle;
  _addr = (IA1 << 6) | (IA0 << 5) | IIC_ADDR_FIXED;
  _subSerialChannel = subUartChannel;
  _rx_buffer_head = 0;
  _rx_buffer_tail = 0;
  memset(_rx_buffer, 0, sizeof(_rx_buffer));
}

DFRobot_IICSerial::~DFRobot_IICSerial() {}

int DFRobot_IICSerial::begin(long unsigned baud, uint8_t format, eCommunicationMode_t mode,
                                eLineBreakOutput_t opt) {
  // esp_log_level_set(TAG, ESP_LOG_VERBOSE);

  _rx_buffer_head = _rx_buffer_tail;
  uint8_t val = 0;
  uint8_t channel = subSerialChnnlSwitch(SUBUART_CHANNEL_1);
  if (readReg(REG_WK2132_GENA, &val, 1) != 1) {
    ESP_LOGV(TAG, "READ BYTEERROR!");
    return ERR_READ;
  }
#ifndef ARDUINO_ARCH_NRF5
  if ((val & 0x80) == 0) {
    ESP_LOGV(TAG, "Read REG_WK2132_GENA  ERROR!");
    return ERR_REGDATA;
  }
#endif
  subSerialChnnlSwitch(channel);
  subSerialConfig(_subSerialChannel);
  ESP_LOGV(TAG, "OK");
  setSubSerialBaudRate(baud);
  setSubSerialConfigReg(format, mode, opt);
  return ERR_OK;
}

void DFRobot_IICSerial::end() { subSerialGlobalRegEnable(_subSerialChannel, rst); }

int DFRobot_IICSerial::available(void) {
  int index;
  uint8_t val = 0;
  sFsrReg_t fsr;
  if (readReg(REG_WK2132_RFCNT, &val, 1) != 1) {
    ESP_LOGV(TAG, "READ BYTE SIZE ERROR!");
    return 0;
  }
  index = (int)val;
  if (index == 0) {
    fsr = readFIFOStateReg();
    if (fsr.rDat == 1) {
      index = 256;
    }
  }
  int result =
      (index + ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) %
                   SERIAL_RX_BUFFER_SIZE);
  return result;
}

int DFRobot_IICSerial::peek(void) {
  int num =
      available() - (((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) %
                     SERIAL_RX_BUFFER_SIZE);
  for (int i = 0; i < num; i++) {
    rx_buffer_index_t j = (rx_buffer_index_t)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    if (j != _rx_buffer_tail) {
      uint8_t val = 0;
      readReg(REG_WK2132_FDAT, &val, 1);
      _rx_buffer[_rx_buffer_head] = val;
      _rx_buffer_head = j;
    } else {
      break;
    }
  }
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  }
  return _rx_buffer[_rx_buffer_tail];
}

int DFRobot_IICSerial::read(void) {
  int num =
      available() - ((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) %
                        SERIAL_RX_BUFFER_SIZE;
  for (int i = 0; i < num; i++) {
    rx_buffer_index_t j = (rx_buffer_index_t)(_rx_buffer_head + 1) % SERIAL_RX_BUFFER_SIZE;
    if (j != _rx_buffer_tail) {
      uint8_t val = 0;
      readReg(REG_WK2132_FDAT, &val, 1);
      _rx_buffer[_rx_buffer_head] = val;
      _rx_buffer_head = j;
    } else {
      break;
    }
  }
  if (_rx_buffer_head == _rx_buffer_tail) {
    return -1;
  }
  unsigned char c = _rx_buffer[_rx_buffer_tail];
  _rx_buffer_tail = (rx_buffer_index_t)(_rx_buffer_tail + 1) % SERIAL_RX_BUFFER_SIZE;
  return c;
}

// size_t DFRobot_IICSerial::write(uint8_t value) {
//   sFsrReg_t fsr;
//   fsr = readFIFOStateReg();
//   if (fsr.tFull == 1) {
//     ESP_LOGV(TAG, "FIFO full!");
//     return -1;
//   }
//   writeReg(REG_WK2132_FDAT, &value, 1);
//   return 1;
// }

void DFRobot_IICSerial::print(const char *str) {
  write((const uint8_t *)str, strlen(str));
}

int DFRobot_IICSerial::write(const uint8_t *pBuf, int size) {
  if (pBuf == NULL) {
    ESP_LOGV(TAG, "pBuf ERROR!! : null pointer");
    return 0;
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;
  sFsrReg_t fsr;
  uint8_t val = 0;
  fsr = readFIFOStateReg();
  if (fsr.tFull == 1) {
    ESP_LOGV(TAG, "FIFO full!");
    return 0;
  }
  writeFIFO(_pBuf, size);
  return size;
}

size_t DFRobot_IICSerial::read(void *pBuf, size_t size) {
  if (pBuf == NULL) {
    ESP_LOGV(TAG, "pBuf ERROR!! : null pointer");
    return 0;
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;
  size =
      available() - (((unsigned int)(SERIAL_RX_BUFFER_SIZE + _rx_buffer_head - _rx_buffer_tail)) %
                     SERIAL_RX_BUFFER_SIZE);
  readFIFO(_pBuf, size);
  return size;
}
void DFRobot_IICSerial::flush(void) {
  sFsrReg_t fsr = readFIFOStateReg();
  while (fsr.tDat == 1)
    ;
}

void DFRobot_IICSerial::subSerialConfig(uint8_t subUartChannel) {
  ESP_LOGV(TAG, "Sub UART clock enable");
  subSerialGlobalRegEnable(subUartChannel, clock);
  ESP_LOGV(TAG, "Software reset sub UART");
  subSerialGlobalRegEnable(subUartChannel, rst);
  ESP_LOGV(TAG, "Sub UART global interrupt enable");
  subSerialGlobalRegEnable(subUartChannel, intrpt);
  ESP_LOGV(TAG, "Sub UART page register setting (default PAGE0)");
  subSerialPageSwitch(page0);
  ESP_LOGV(TAG, "Sub interrupt setting");
  sSierReg_t sier = {
      .rFTrig = 0x01, .rxOvt = 0x01, .tfTrig = 0x01, .tFEmpty = 0x01, .rsv = 0x00, .fErr = 0x01};
  subSerialRegConfig(REG_WK2132_SIER, &sier);
  ESP_LOGV(TAG, "enable transmit/receive FIFO");
  sFcrReg_t fcr = {
      .rfRst = 0x01, .tfRst = 0x00, .rfEn = 0x01, .tfEn = 0x01, .rfTrig = 0x00, .tfTrig = 0x00};
  subSerialRegConfig(REG_WK2132_FCR, &fcr);
  ESP_LOGV(TAG, "Sub UART reiceive/transmit enable");
  sScrReg_t scr = {.rxEn = 0x01, .txEn = 0x01, .sleepEn = 0x01, .rsv = 0x00};
  subSerialRegConfig(REG_WK2132_SCR, &scr);
}

void DFRobot_IICSerial::subSerialGlobalRegEnable(uint8_t subUartChannel, eGlobalRegType_t type) {
  if (subUartChannel > SUBUART_CHANNEL_ALL) {
    ESP_LOGV(TAG, "SUBSERIAL CHANNEL NUMBER ERROR!");
    return;
  }
  uint8_t val = 0;
  uint8_t regAddr = getGlobalRegType(type);
  uint8_t channel = subSerialChnnlSwitch(SUBUART_CHANNEL_1);
  ESP_LOGV(TAG, "reg: %.2x", regAddr);
  if (readReg(regAddr, &val, 1) != 1) {
    ESP_LOGV(TAG, "READ BYTE SIZE ERROR!");
    return;
  }
  ESP_LOGV(TAG, "before: %.2x", val);
  switch (subUartChannel) {
  case SUBUART_CHANNEL_1:
    val |= 0x01;
    break;
  case SUBUART_CHANNEL_2:
    val |= 0x02;
    break;
  default:
    val |= 0x03;
    break;
  }
  writeReg(regAddr, &val, 1);
  readReg(regAddr, &val, 1);
  ESP_LOGV(TAG, "after: %.2x", val);
  subSerialChnnlSwitch(channel);
}

void DFRobot_IICSerial::subSerialPageSwitch(ePageNumber_t page) {
  if (page >= pageTotal) {
    return;
  }
  uint8_t val = 0;
  if (readReg(REG_WK2132_SPAGE, &val, 1) != 1) {
    ESP_LOGV(TAG, "READ BYTE SIZE ERROR!");
    return;
  }
  switch (page) {
  case page0:
    val &= 0xFE;
    break;
  case page1:
    val |= 0x01;
    break;
  default:
    break;
  }
  ESP_LOGV(TAG, "before: %.2x", val);
  writeReg(REG_WK2132_SPAGE, &val, 1);
  readReg(REG_WK2132_SPAGE, &val, 1);
  ESP_LOGV(TAG, "after: %.2x", val);
}

void DFRobot_IICSerial::subSerialRegConfig(uint8_t reg, void *pValue) {
  uint8_t val = 0;
  readReg(reg, &val, 1);
  ESP_LOGV(TAG, "before: %.2x", val);
  val |= *(uint8_t *)pValue;
  writeReg(reg, &val, 1);
  readReg(reg, &val, 1);
  ESP_LOGV(TAG, "after: %.2x", val);
}

uint8_t DFRobot_IICSerial::getGlobalRegType(eGlobalRegType_t type) {
  if ((type < clock) || (type > intrpt)) {
    ESP_LOGV(TAG, "Global Reg Type Error!");
    return 0;
  }
  uint8_t regAddr = 0;
  switch (type) {
  case clock:
    regAddr = REG_WK2132_GENA;
    break;
  case rst:
    regAddr = REG_WK2132_GRST;
    break;
  default:
    regAddr = REG_WK2132_GIER;
    break;
  }
  return regAddr;
}

void DFRobot_IICSerial::setSubSerialBaudRate(unsigned long baud) {
  uint8_t scr = 0x00, clear = 0x00;
  readReg(REG_WK2132_SCR, &scr, 1);
  subSerialRegConfig(REG_WK2132_SCR, &clear);
  uint8_t baud1 = 0, baud0 = 0, baudPres = 0;
  uint16_t valIntger = FOSC / (baud * 16) - 1;
  uint16_t valDecimal = (FOSC % (baud * 16)) / (baud * 16);
  baud1 = (uint8_t)(valIntger >> 8);
  baud0 = (uint8_t)(valIntger & 0x00ff);
  while (valDecimal > 0x0A) {
    valDecimal /= 0x0A;
  }
  baudPres = (uint8_t)(valDecimal);
  subSerialPageSwitch(page1);
  subSerialRegConfig(REG_WK2132_BAUD1, &baud1);
  subSerialRegConfig(REG_WK2132_BAUD0, &baud0);
  subSerialRegConfig(REG_WK2132_PRES, &baudPres);

  readReg(REG_WK2132_BAUD1, &baud1, 1);
  readReg(REG_WK2132_BAUD0, &baud0, 1);
  readReg(REG_WK2132_PRES, &baudPres, 1);
  ESP_LOGV(TAG, "%.2x", baud1);
  ESP_LOGV(TAG, "%.2x", baud0);
  ESP_LOGV(TAG, "%.2x", baudPres);
  subSerialPageSwitch(page0);
  subSerialRegConfig(REG_WK2132_SCR, &scr);
}

void DFRobot_IICSerial::setSubSerialConfigReg(uint8_t format, eCommunicationMode_t mode,
                                                 eLineBreakOutput_t opt) {
  uint8_t _mode = (uint8_t)mode;
  uint8_t _opt = (uint8_t)opt;
  uint8_t val = 0;
  _addr = updateAddr(_addr, _subSerialChannel, OBJECT_REGISTER);
  if (readReg(REG_WK2132_LCR, &val, 1) != 1) {
    ESP_LOGV(TAG, "Read Byte ERROR！");
    return;
  }
  ESP_LOGV(TAG, "before: %.2x", val);
  sLcrReg_t lcr = *((sLcrReg_t *)(&val));
  lcr.format = format;
  lcr.irEn = _mode;
  lcr.lBreak = _opt;
  val = *(uint8_t *)&lcr;
  writeReg(REG_WK2132_LCR, &val, 1);
  readReg(REG_WK2132_LCR, &val, 1);
  ESP_LOGV(TAG, "before: %.2x", val);
}

uint8_t DFRobot_IICSerial::updateAddr(uint8_t pre, uint8_t subUartChannel, uint8_t obj) {
  sIICAddr_t addr = {.type = obj, .uart = subUartChannel, .addrPre = (uint8_t)((int)pre >> 3)};
  return *(uint8_t *)&addr;
}

DFRobot_IICSerial::sFsrReg_t DFRobot_IICSerial::readFIFOStateReg() {
  sFsrReg_t fsr;
  readReg(REG_WK2132_FSR, &fsr, sizeof(fsr));
  return fsr;
}

uint8_t DFRobot_IICSerial::subSerialChnnlSwitch(uint8_t subUartChannel) {
  uint8_t channel = _subSerialChannel;
  _subSerialChannel = subUartChannel;
  return channel;
}

void DFRobot_IICSerial::turnOffClock() {
  // uint8_t valGENA = 0;
  // // GENA
  // if (readReg(REG_WK2132_GENA, &valGENA, 1) != 1) {
  //   Serial.println("READ REG_WK2132_GENA ERROR!");
  //   return;
  // }
  // Serial.print("BEFORE CHANGE GENA: ");
  // Serial.println(valGENA, BIN);
  //
  // valGENA &= 0B11111100;
  // writeReg(REG_WK2132_GENA, &valGENA, 1);
  // if (readReg(REG_WK2132_GENA, &valGENA, 1) != 1) {
  //   Serial.println("READ REG_WK2132_GENA ERROR!");
  //   return;
  // }
  // Serial.print("BEFORE CHANGE GENA: ");
  // Serial.println(valGENA, BIN);
}

void DFRobot_IICSerial::printAllRegsForCurrentCh() {

  // Serial.print("CH: ");
  // Serial.println(_subSerialChannel);
  // uint8_t valGENA = 0;
  // // GENA
  // if (readReg(REG_WK2132_GENA, &valGENA, 1) != 1) {
  //   Serial.println("READ REG_WK2132_GENA ERROR!");
  //   return;
  // }
  // Serial.print("GENA: ");
  // Serial.println(valGENA, BIN);
  //
  // uint8_t valGRST = 0;
  // // GRST
  // if (readReg(REG_WK2132_GRST, &valGRST, 1) != 1) {
  //   Serial.println("READ REG_WK2132_GRST ERROR!");
  //   return;
  // }
  // Serial.print("GRST: ");
  // Serial.println(valGRST, BIN);
  //
  // uint8_t valGMUT = 0;
  // // GMUT
  // if (readReg(REG_WK2132_GMUT, &valGMUT, 1) != 1) {
  //   Serial.println("READ REG_WK2132_GMUT ERROR!");
  //   return;
  // }
  // Serial.print("GMUT: ");
  // Serial.println(valGMUT, BIN);
  //
  // uint8_t valGIER = 0;
  // // GIER
  // if (readReg(REG_WK2132_GIER, &valGIER, 1) != 1) {
  //   Serial.println("READ REG_WK2132_GIER ERROR!");
  //   return;
  // }
  // Serial.print("GIER: ");
  // Serial.println(valGIER, BIN);
  //
  // uint8_t valGIFR = 0;
  // // GIFR
  // if (readReg(REG_WK2132_GIFR, &valGIFR, 1) != 1) {
  //   Serial.println("READ REG_WK2132_GIFR ERROR!");
  //   return;
  // }
  // Serial.print("GIFR: ");
  // Serial.println(valGIFR, BIN);
  //
  // uint8_t valSCR = 0;
  // // SCR
  // if (readReg(REG_WK2132_SCR, &valSCR, 1) != 1) {
  //   Serial.println("READ REG_WK2132_SCR ERROR!");
  //   return;
  // }
  // Serial.print("SCR: ");
  // Serial.println(valSCR, BIN);
  //
  // uint8_t valLCR = 0;
  // // LCR
  // if (readReg(REG_WK2132_LCR, &valLCR, 1) != 1) {
  //   Serial.println("READ REG_WK2132_LCR ERROR!");
  //   return;
  // }
  // Serial.print("LCR: ");
  // Serial.println(valLCR, BIN);
  //
  // uint8_t valFCR = 0;
  // // FCR
  // if (readReg(REG_WK2132_FCR, &valFCR, 1) != 1) {
  //   Serial.println("READ REG_WK2132_FCR ERROR!");
  //   return;
  // }
  // Serial.print("FCR: ");
  // Serial.println(valFCR, BIN);
  //
  // uint8_t valSIER = 0;
  // // SIER
  // if (readReg(REG_WK2132_SIER, &valSIER, 1) != 1) {
  //   Serial.println("READ REG_WK2132_SIER ERROR!");
  //   return;
  // }
  // Serial.print("SIER: ");
  // Serial.println(valSIER, BIN);
  //
  // uint8_t valSIFR = 0;
  // // SIFR
  // if (readReg(REG_WK2132_SIFR, &valSIFR, 1) != 1) {
  //   Serial.println("READ REG_WK2132_SIFR ERROR!");
  //   return;
  // }
  // Serial.print("SIFR: ");
  // Serial.println(valSIFR, BIN);
  //
  // uint8_t valTFCNT = 0;
  // // TFCNT
  // if (readReg(REG_WK2132_TFCNT, &valTFCNT, 1) != 1) {
  //   Serial.println("READ REG_WK2132_TFCNT ERROR!");
  //   return;
  // }
  // Serial.print("TFCNT: ");
  // Serial.println(valTFCNT, BIN);
  //
  // uint8_t valRFCNT = 0;
  // // RFCNT
  // if (readReg(REG_WK2132_RFCNT, &valRFCNT, 1) != 1) {
  //   Serial.println("READ REG_WK2132_RFCNT ERROR!");
  //   return;
  // }
  // Serial.print("RFCNT: ");
  // Serial.println(valRFCNT, BIN);
  //
  // uint8_t valFSR = 0;
  // // FSR
  // if (readReg(REG_WK2132_FSR, &valFSR, 1) != 1) {
  //   Serial.println("READ REG_WK2132_FSR ERROR!");
  //   return;
  // }
  // Serial.print("FSR: ");
  // Serial.println(valFSR, BIN);
  //
  // uint8_t valLSR = 0;
  // // LSR
  // if (readReg(REG_WK2132_LSR, &valLSR, 1) != 1) {
  //   Serial.println("READ REG_WK2132_LSR ERROR!");
  //   return;
  // }
  // Serial.print("LSR: ");
  // Serial.println(valLSR, BIN);
  //
  // uint8_t valFDAT = 0;
  // // FDAT
  // if (readReg(REG_WK2132_FDAT, &valFDAT, 1) != 1) {
  //   Serial.println("READ REG_WK2132_FDAT ERROR!");
  //   return;
  // }
  // Serial.print("FDAT: ");
  // Serial.println(valFDAT, BIN);
}

bool DFRobot_IICSerial::isChannelInSleep(uint8_t subUartChannel) {
  uint8_t val = 0;

  if (readReg(REG_WK2132_GRST, &val, 1) != 1) {
    // Serial.println("READ BYTE SIZE ERROR!");
    return false;
  }
  uint8_t sleepBit;

  switch (subUartChannel) {
  case SUBUART_CHANNEL_1:
    sleepBit = 0B10000;
    break;

  case SUBUART_CHANNEL_2:
  default:
    sleepBit = 0B100000;
    break;
  }

  return (val & sleepBit) > 0;
}

void DFRobot_IICSerial::prepareSleep() {
  sScrReg_t scr = {.rxEn = 0x00, .txEn = 0x00, .sleepEn = 0x01, .rsv = 0x00};
  subSerialRegConfig(REG_WK2132_SCR, &scr);
}

void DFRobot_IICSerial::sleep() {}

void DFRobot_IICSerial::wakeup() {}

void DFRobot_IICSerial::writeReg(uint8_t reg, const void *pBuf, size_t size) {
  if (pBuf == NULL) {
    ESP_LOGV(TAG, "pBuf ERROR!! : null pointer");
    return;
  }
  _addr = updateAddr(_addr, _subSerialChannel, OBJECT_REGISTER);
  i2c_master_dev_handle_t dev_handle = getDeviceHandle(_addr);
  if (!dev_handle) {
    ESP_LOGV(TAG, "Failed get device hanlde");
    return;
  }

  const uint8_t *data = (const uint8_t *)pBuf;

  // Compile transmit payload
  uint8_t *txBuf = new uint8_t[size + 1];
  txBuf[0] = reg;                // Register
  memcpy(&txBuf[1], data, size); // data

  // Transmit reg byte first, then data in a single call
  esp_err_t err = i2c_master_transmit(dev_handle, txBuf, (size + 1), 500); // Send register
  if (err != ESP_OK) {
    ESP_LOGV(TAG, "writeReg() i2c transmit failed");
    delete[] txBuf;
    return;
  }

  delete[] txBuf;
}

uint8_t DFRobot_IICSerial::readReg(uint8_t reg, void *pBuf, size_t size) {
  if (pBuf == NULL) {
    ESP_LOGV(TAG, "pBuf ERROR!! : null pointer");
    return 0;
  }
  _addr = updateAddr(_addr, _subSerialChannel, OBJECT_REGISTER);
  _addr &= 0xFE;
  i2c_master_dev_handle_t dev_handle = getDeviceHandle(_addr);
  if (!dev_handle) {
    ESP_LOGV(TAG, "Failed get device hanlde");
    return 0;
  }

  // uint8_t *_pBuf = (uint8_t *)pBuf;

  // Step 1: Write register address
  esp_err_t err = i2c_master_transmit(dev_handle, &reg, 1, 500);
  if (err != ESP_OK) {
    ESP_LOGV(TAG, "Failed to send reg 0x%02X to device 0x%02X", reg, _addr);
    return 0;
  }

  // Step 2: Read response
  err = i2c_master_receive(dev_handle, (uint8_t *)pBuf, size, 500);
  if (err != ESP_OK) {
    ESP_LOGV(TAG, "Failed to read from device 0x%02X", _addr);
    return 0;
  }

  return size;
}

uint8_t DFRobot_IICSerial::readFIFO(void *pBuf, size_t size) {
  if (pBuf == NULL) {
    ESP_LOGV(TAG, "pBuf ERROR!! : null pointer");
    return 0;
  }
  _addr = updateAddr(_addr, _subSerialChannel, OBJECT_FIFO);
  i2c_master_dev_handle_t dev_handle = getDeviceHandle(_addr);
  if (!dev_handle) {
    return 0;
  }

  uint8_t *buf = (uint8_t *)pBuf;
  size_t left = size;
  size_t num = 0;

  while (left > 0) {
    num = (left > IIC_BUFFER_SIZE) ? IIC_BUFFER_SIZE : left;

    // Dummy write (just beginTransmission + endTransmission in Arduino)
    esp_err_t err = i2c_master_transmit(dev_handle, NULL, 0, 500);
    if (err != ESP_OK) {
      ESP_LOGV(TAG, "Failed to initiate FIFO read from 0x%02X", _addr);
      return 0;
    }

    // Read num bytes
    err = i2c_master_receive(dev_handle, buf, num, 500);
    if (err != ESP_OK) {
      ESP_LOGV(TAG, "Failed to read FIFO data from 0x%02X", _addr);
      return 0;
    }

    buf += num;
    left -= num;
  }

  return size;
}

void DFRobot_IICSerial::writeFIFO(void *pBuf, size_t size) {
  if (pBuf == NULL) {
    ESP_LOGV(TAG, "pBuf ERROR!! : null pointer");
    return;
  }
  _addr = updateAddr(_addr, _subSerialChannel, OBJECT_FIFO);
  i2c_master_dev_handle_t dev_handle = getDeviceHandle(_addr);
  if (!dev_handle) {
    return;
  }

  const uint8_t *buf = (const uint8_t *)pBuf;
  size_t left = size;
  size_t chunk = 0;

  while (left > 0) {
    chunk = (left > IIC_BUFFER_SIZE) ? IIC_BUFFER_SIZE : left;

    esp_err_t err = i2c_master_transmit(dev_handle, buf, chunk, 500);
    if (err != ESP_OK) {
      ESP_LOGV(TAG, "FIFO write failed to 0x%02X", _addr);
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
    buf += chunk;
    left -= chunk;
  }
}

i2c_master_dev_handle_t DFRobot_IICSerial::getDeviceHandle(uint8_t addr) {
  // Iterate through device_map to find the device by address
  for (int i = 0; i < MAX_WK2132_ADDR; i++) {
    if (dev_handle_map[i].address == addr) {
      // Device already initialized, return handle
      return dev_handle_map[i].dev_handle;
    }
  }

  // If the device is not found, initialize it
  for (int i = 0; i < MAX_WK2132_ADDR; i++) {
    if (dev_handle_map[i].address == 0) { // If empty slot
      // Configure device on the bus
      i2c_device_config_t dev_cfg = {
          .dev_addr_length = I2C_ADDR_BIT_LEN_7,
          .device_address = addr,
          .scl_speed_hz = 100000,
      };
      dev_handle_map[i].address = addr;

      esp_err_t ret =
          i2c_master_bus_add_device(_i2c_bus_handle, &dev_cfg, &dev_handle_map[i].dev_handle);
      if (ret != ESP_OK) {
        ESP_LOGV(TAG, "Failed to add device at address %d", addr);
        return NULL;
      }

      ESP_LOGV(TAG, "New address created %d", addr);
      return dev_handle_map[i].dev_handle;
    }
  }

  ESP_LOGV(TAG, "No space for new device at address %d", addr);
  return NULL; // No empty slot available for new device
}
