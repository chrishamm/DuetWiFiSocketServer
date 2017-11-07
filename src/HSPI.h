/* 
  SPI.h - SPI library for esp8266

  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the esp8266 core for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef _HSPI_H_INCLUDED
#define _HSPI_H_INCLUDED

#include <Arduino.h>
#include <stdlib.h>

const uint8_t SPI_MODE0 = 0x00; ///<  CPOL: 0  CPHA: 0
const uint8_t SPI_MODE1 = 0x01; ///<  CPOL: 0  CPHA: 1
const uint8_t SPI_MODE2 = 0x10; ///<  CPOL: 1  CPHA: 0
const uint8_t SPI_MODE3 = 0x11; ///<  CPOL: 1  CPHA: 1

class SPISettings {
public:
  SPISettings() :_clock(1000000), _bitOrder(LSBFIRST), _dataMode(SPI_MODE0){}
  SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode) :_clock(clock), _bitOrder(bitOrder), _dataMode(dataMode){}
  uint32_t _clock;
  uint8_t  _bitOrder;
  uint8_t  _dataMode;
};

class HSPIClass {
public:
  HSPIClass();
  void begin();
  void end();
  void setHwCs(bool use);
  void setBitOrder(uint8_t bitOrder);  
  void setDataMode(uint8_t dataMode);
  void setDataBits(uint16_t bits);
  void setFrequency(uint32_t freq);
  void setClockDivider(uint32_t clockDiv);
  void beginTransaction();
  uint32_t transfer32(uint32_t data);
  void transferDwords(const uint32_t * out, uint32_t * in, uint32_t size);
  void endTransaction(void);

private:
  bool useHwCs;
  void transferDwords_(const uint32_t * out, uint32_t * in, uint8_t size);
};

#endif
