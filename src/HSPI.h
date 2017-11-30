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

#include <stdlib.h>
#include <esp/spi.h>

class SPISettings {
public:
  SPISettings() :_clock(1000000), _bitOrder(SPI_LITTLE_ENDIAN), _dataMode(SPI_MODE0){}
  SPISettings(uint32_t clock, _spi_endianness_t bitOrder, _spi_mode_t dataMode) :_clock(clock), _bitOrder(bitOrder), _dataMode(dataMode){}
  uint32_t _clock;
  _spi_endianness_t  _bitOrder;
  _spi_mode_t  _dataMode;
};

class HSPIClass {
public:
  void begin();
  void end();
  void setBitOrder(_spi_endianness_t bitOrder);
  void setDataMode(_spi_mode_t dataMode);
  void setFrequency(uint32_t freq);
  void setClockDivider(uint32_t clockDiv);
  void beginTransaction(SPISettings settings);
  void beginTransaction();
  uint8_t transfer(uint8_t data);
  uint16_t transfer16(uint16_t data);
  uint32_t transfer32(uint32_t data);
  void write(uint8_t data);
  void write16(uint16_t data);
  void write16(uint16_t data, bool msb);
  void write32(uint32_t data);
  void write32(uint32_t data, bool msb);
//  void writeDword(uint32_t data);
  void writeBytes(const uint8_t * data, uint32_t size);
  void writeDwords(const uint32_t * data, uint32_t size);
  void writePattern(const uint8_t * data, uint8_t size, uint32_t repeat);
  void transferBytes(const uint8_t * out, uint8_t * in, uint32_t size);
  void transferDwords(const uint32_t * out, uint32_t * in, uint32_t size);
  void endTransaction(void);
private:
  void writeBytes_(const uint8_t * data, uint8_t size);
  void writeDwords_(const uint32_t * data, uint8_t size);
  void writePattern_(const uint8_t * data, uint8_t size, uint8_t repeat);
  void transferBytes_(const uint8_t * out, uint8_t * in, uint8_t size);
  void transferDwords_(const uint32_t * out, uint32_t * in, uint8_t size);
public:
  void setDataBits(uint16_t bits);
};

#endif
