/* 
 SPI.cpp - SPI library for esp8266

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

#include "HSPI.h"

typedef union {
        uint32_t regValue;
        struct {
                unsigned regL :6;
                unsigned regH :6;
                unsigned regN :6;
                unsigned regPre :13;
                unsigned regEQU :1;
        };
} spiClk_t;

HSPIClass::HSPIClass() {
    useHwCs = false;
}

void HSPIClass::begin() {
    pinMode(SCK, SPECIAL);  ///< GPIO14
    pinMode(MISO, SPECIAL); ///< GPIO12
    pinMode(MOSI, SPECIAL); ///< GPIO13

    SPI1C = 0;
    setFrequency(1000000); ///< 1MHz
    SPI1U = SPIUMOSI | SPIUDUPLEX | SPIUSSE /*| SPIUWRBYO | SPIURDBYO*/;    // slave SPI mode 0
    SPI1U1 = (7 << SPILMOSI) | (7 << SPILMISO);
    SPI1C1 = 0;
}

void HSPIClass::end() {
    pinMode(SCK, INPUT);
    pinMode(MISO, INPUT);
    pinMode(MOSI, INPUT);
    if(useHwCs) {
        pinMode(SS, INPUT);
    }
}

// Begin a transaction without changing settings
void ICACHE_RAM_ATTR HSPIClass::beginTransaction() {
    while(SPI1CMD & SPIBUSY) {}
}

void ICACHE_RAM_ATTR HSPIClass::endTransaction() {
}

void HSPIClass::setDataMode(uint8_t dataMode) {

    /**
     SPI_MODE0 0x00 - CPOL: 0  CPHA: 0
     SPI_MODE1 0x01 - CPOL: 0  CPHA: 1
     SPI_MODE2 0x10 - CPOL: 1  CPHA: 0
     SPI_MODE3 0x11 - CPOL: 1  CPHA: 1
     */

    bool CPOL = (dataMode & 0x10); ///< CPOL (Clock Polarity)
    bool CPHA = (dataMode & 0x01); ///< CPHA (Clock Phase)

    if(CPHA) {
        SPI1U |= (SPIUSME | SPIUSSE);
    } else {
        SPI1U &= ~(SPIUSME | SPIUSSE);
    }

    if(CPOL) {
        //todo How set CPOL???
    }

}

void HSPIClass::setBitOrder(uint8_t bitOrder) {
    if(bitOrder == MSBFIRST) {
        SPI1C &= ~(SPICWBO | SPICRBO);
    } else {
        SPI1C |= (SPICWBO | SPICRBO);
    }
}

/**
 * calculate the Frequency based on the register value
 * @param reg
 * @return
 */
static uint32_t ClkRegToFreq(spiClk_t * reg) {
    return (ESP8266_CLOCK / ((reg->regPre + 1) * (reg->regN + 1)));
}

void HSPIClass::setFrequency(uint32_t freq) {
    static uint32_t lastSetFrequency = 0;
    static uint32_t lastSetRegister = 0;

    if(freq >= ESP8266_CLOCK) {
        setClockDivider(0x80000000);
        return;
    }

    if(lastSetFrequency == freq && lastSetRegister == SPI1CLK) {
        // do nothing (speed optimization)
        return;
    }

    const spiClk_t minFreqReg = { 0x7FFFF000 };
    uint32_t minFreq = ClkRegToFreq((spiClk_t*) &minFreqReg);
    if(freq < minFreq) {
        // use minimum possible clock
        setClockDivider(minFreqReg.regValue);
        lastSetRegister = SPI1CLK;
        lastSetFrequency = freq;
        return;
    }

    uint8_t calN = 1;

    spiClk_t bestReg = { 0 };
    int32_t bestFreq = 0;

    // find the best match
    while(calN <= 0x3F) { // 0x3F max for N

        spiClk_t reg = { 0 };
        int32_t calFreq;
        int32_t calPre;
        int8_t calPreVari = -2;

        reg.regN = calN;

        while(calPreVari++ <= 1) { // test different variants for Pre (we calculate in int so we miss the decimals, testing is the easyest and fastest way)
            calPre = (((ESP8266_CLOCK / (reg.regN + 1)) / freq) - 1) + calPreVari;
            if(calPre > 0x1FFF) {
                reg.regPre = 0x1FFF; // 8191
            } else if(calPre <= 0) {
                reg.regPre = 0;
            } else {
                reg.regPre = calPre;
            }

            reg.regL = ((reg.regN + 1) / 2);
            // reg.regH = (reg.regN - reg.regL);

            // test calculation
            calFreq = ClkRegToFreq(&reg);
            //os_printf("-----[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d = %d\n", reg.regValue, freq, reg.regEQU, reg.regPre, reg.regN, reg.regH, reg.regL, calFreq);

            if(calFreq == (int32_t) freq) {
                // accurate match use it!
                memcpy(&bestReg, &reg, sizeof(bestReg));
                break;
            } else if(calFreq < (int32_t) freq) {
                // never go over the requested frequency
                if(abs(freq - calFreq) < abs(freq - bestFreq)) {
                    bestFreq = calFreq;
                    memcpy(&bestReg, &reg, sizeof(bestReg));
                }
            }
        }
        if(calFreq == (int32_t) freq) {
            // accurate match use it!
            break;
        }
        calN++;
    }

    // os_printf("[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d\t - Real Frequency: %d\n", bestReg.regValue, freq, bestReg.regEQU, bestReg.regPre, bestReg.regN, bestReg.regH, bestReg.regL, ClkRegToFreq(&bestReg));

    setClockDivider(bestReg.regValue);
    lastSetRegister = SPI1CLK;
    lastSetFrequency = freq;

}

void HSPIClass::setClockDivider(uint32_t clockDiv) {
    if(clockDiv == 0x80000000) {
        GPMUX |= (1 << 9); // Set bit 9 if sysclock required
    } else {
        GPMUX &= ~(1 << 9);
    }
    SPI1CLK = clockDiv;
}

void HSPIClass::setDataBits(uint16_t bits) {
    const uint32_t mask = ~((SPIMMOSI << SPILMOSI) | (SPIMMISO << SPILMISO));
    bits--;
    SPI1U1 = ((SPI1U1 & mask) | ((bits << SPILMOSI) | (bits << SPILMISO)));
}

uint32_t ICACHE_RAM_ATTR HSPIClass::transfer32(uint32_t data)
{
    while(SPI1CMD & SPIBUSY) {}
    // Set to 32Bits transfer
    setDataBits(32);
	// LSBFIRST Byte first
	SPI1W0 = data;
	SPI1CMD |= SPIBUSY;
    while(SPI1CMD & SPIBUSY) {}
    return SPI1W0;
}

/**
 * @param out uint32_t *
 * @param in  uint32_t *
 * @param size uint32_t
 */
void ICACHE_RAM_ATTR HSPIClass::transferDwords(const uint32_t * out, uint32_t * in, uint32_t size) {
    while(size != 0) {
        if (size > 16) {
            transferDwords_(out, in, 16);
            size -= 16;
            if(out) out += 16;
            if(in) in += 16;
        } else {
            transferDwords_(out, in, size);
            size = 0;
        }
    }
}

void ICACHE_RAM_ATTR HSPIClass::transferDwords_(const uint32_t * out, uint32_t * in, uint8_t size) {
    while(SPI1CMD & SPIBUSY) {}

    // Set in/out Bits to transfer
    setDataBits(size * 32);

    volatile uint32_t * fifoPtr = &SPI1W0;
    uint8_t dataSize = size;

    if (out != nullptr) {
        while(dataSize != 0) {
            *fifoPtr++ = *out++;
            dataSize--;
        }
    } else {
        // no out data, so fill with dummy data
        while(dataSize != 0) {
            *fifoPtr++ = 0xFFFFFFFF;
            dataSize--;
        }
    }

    SPI1CMD |= SPIBUSY;
    while(SPI1CMD & SPIBUSY) {}

    if (in != nullptr) {
        volatile uint32_t * fifoPtrRd = &SPI1W0;
        while(size != 0) {
            *in++ = *fifoPtrRd++;
            size--;
        }
    }
}

// End
