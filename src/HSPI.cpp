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

#include <esp/gpio.h>
#include <esp/spi_regs.h>
#include <esp/iomux_regs.h>

#include <FreeRTOSConfig.h>

#include <cstring>

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

#define _SPI1_MISO_GPIO 12
#define _SPI1_MOSI_GPIO 13
#define _SPI1_SCK_GPIO  14
#define _SPI1_CS0_GPIO  15

#define _SPI1_FUNC IOMUX_FUNC(2)


void HSPIClass::begin() {
    gpio_set_iomux_function(_SPI1_MISO_GPIO, _SPI1_FUNC);	///< GPIO12
    gpio_set_iomux_function(_SPI1_MOSI_GPIO, _SPI1_FUNC);	///< GPIO13
    gpio_set_iomux_function(_SPI1_SCK_GPIO, _SPI1_FUNC);	///< GPIO14

	SPI(1).CTRL0 = 0;
    setFrequency(1000000); ///< 1MHz
	SPI(1).USER0 = SPI_USER0_MOSI | SPI_USER0_DUPLEX | SPI_USER0_CLOCK_IN_EDGE /*| SPIUWRBYO | SPIURDBYO*/;    // slave SPI mode 0
	SPI(1).USER1 = (7 << SPI_USER1_MOSI_BITLEN_S) | (7 << SPI_USER1_MISO_BITLEN_S);
	SPI(1).CTRL1 = 0;
}

void HSPIClass::end() {
	gpio_enable(_SPI1_MISO_GPIO, GPIO_INPUT);
	gpio_enable(_SPI1_MOSI_GPIO, GPIO_INPUT);
	gpio_enable(_SPI1_SCK_GPIO, GPIO_INPUT);
}

void HSPIClass::beginTransaction(SPISettings settings) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    setFrequency(settings._clock);
    setBitOrder(settings._bitOrder);
    setDataMode(settings._dataMode);
}

// Begin a transaction without changing settings
void HSPIClass::beginTransaction() {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
}

void HSPIClass::endTransaction() {
}

void HSPIClass::setDataMode(_spi_mode_t dataMode) {

    /**
     SPI_MODE0 0x00 - CPOL: 0  CPHA: 0
     SPI_MODE1 0x01 - CPOL: 0  CPHA: 1
     SPI_MODE2 0x10 - CPOL: 1  CPHA: 0
     SPI_MODE3 0x11 - CPOL: 1  CPHA: 1
     */

#if 0
    bool CPOL = (dataMode & 0x10); ///< CPOL (Clock Polarity)
#endif
    bool CPHA = (dataMode & 0x01); ///< CPHA (Clock Phase)

    if (CPHA)
    {
    	SPI(1).USER0 |= (SPI_USER0_CLOCK_OUT_EDGE | SPI_USER0_CLOCK_IN_EDGE);
    }
    else
    {
    	SPI(1).USER0 &= ~(SPI_USER0_CLOCK_OUT_EDGE | SPI_USER0_CLOCK_IN_EDGE);
    }
}

void HSPIClass::setBitOrder(_spi_endianness_t bitOrder) {
    if (bitOrder == SPI_BIG_ENDIAN)
    {
    	SPI(1).CMD &= ~(SPI_CTRL0_WR_BIT_ORDER | SPI_CTRL0_RD_BIT_ORDER);
    }
    else
    {
    	SPI(1).CMD |= (SPI_CTRL0_WR_BIT_ORDER | SPI_CTRL0_RD_BIT_ORDER);
    }
}

/**
 * calculate the Frequency based on the register value
 * @param reg
 * @return
 */
static uint32_t ClkRegToFreq(spiClk_t * reg) {
    return (configCPU_CLOCK_HZ / ((reg->regPre + 1) * (reg->regN + 1)));
}

void HSPIClass::setFrequency(uint32_t freq) {
    static uint32_t lastSetFrequency = 0;
    static uint32_t lastSetRegister = 0;

    if (freq >= configCPU_CLOCK_HZ)
    {
        setClockDivider(0x80000000);
        return;
    }

    if (lastSetFrequency == freq && lastSetRegister == SPI(1).CLOCK)
    {
        // do nothing (speed optimization)
        return;
    }

    const spiClk_t minFreqReg = { 0x7FFFF000 };
    uint32_t minFreq = ClkRegToFreq((spiClk_t*) &minFreqReg);
    if (freq < minFreq)
    {
        // use minimum possible clock
        setClockDivider(minFreqReg.regValue);
        lastSetRegister = SPI(1).CLOCK;
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
            calPre = (((configCPU_CLOCK_HZ / (reg.regN + 1)) / freq) - 1) + calPreVari;
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
                if (abs((int32_t)freq - calFreq) < abs((int32_t)freq - bestFreq)) {
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

    //os_printf("[0x%08X][%d]\t EQU: %d\t Pre: %d\t N: %d\t H: %d\t L: %d\t - Real Frequency: %d\n", bestReg.regValue, freq, bestReg.regEQU, bestReg.regPre, bestReg.regN, bestReg.regH, bestReg.regL, ClkRegToFreq(&bestReg));

    setClockDivider(bestReg.regValue);
    lastSetRegister = SPI(1).CLOCK;
    lastSetFrequency = freq;
}

void HSPIClass::setClockDivider(uint32_t clockDiv) {
    if (clockDiv == 0x80000000)
    {
		IOMUX.CONF |= (1 << 9); // Set bit 9 if sysclock required
    }
    else
    {
    	IOMUX.CONF &= ~(1 << 9);
    }
    SPI(1).CLOCK = clockDiv;
}

void HSPIClass::setDataBits(uint16_t bits) {
    const uint32_t mask = ~((SPI_USER1_MOSI_BITLEN_M << SPI_USER1_MOSI_BITLEN_S) | (SPI_USER1_MISO_BITLEN_M << SPI_USER1_MISO_BITLEN_S));
    bits--;
    SPI(1).USER1 = ((SPI(1).USER1 & mask) | ((bits << SPI_USER1_MOSI_BITLEN_S) | (bits << SPI_USER1_MISO_BITLEN_S)));
}

uint8_t HSPIClass::transfer(uint8_t data) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // reset to 8Bit mode
    setDataBits(8);
    SPI(1).W[0] = data;
    SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    return (uint8_t) (SPI(1).W[0] & 0xff);
}

uint16_t HSPIClass::transfer16(uint16_t data) {
    union {
            uint16_t val;
            struct {
                    uint8_t lsb;
                    uint8_t msb;
            };
    } in, out;
    in.val = data;

    if ((SPI(1).CTRL0 & (SPI_CTRL0_WR_BIT_ORDER | SPI_CTRL0_RD_BIT_ORDER)) != 0) {
        //MSBFIRST
        out.msb = transfer(in.msb);
        out.lsb = transfer(in.lsb);
    } else {
        //LSBFIRST
        out.lsb = transfer(in.lsb);
        out.msb = transfer(in.msb);
    }
    return out.val;
}

uint32_t HSPIClass::transfer32(uint32_t data)
{
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // Set to 32Bits transfer
    setDataBits(32);
	// LSBFIRST Byte first
    SPI(1).W[0] = data;
    SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    return SPI(1).W[0];
}

void HSPIClass::write(uint8_t data) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // reset to 8Bit mode
    setDataBits(8);
    SPI(1).W[0] = data;
    SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
}

void HSPIClass::write16(uint16_t data) {
    write16(data, (SPI(1).CTRL0 & (SPI_CTRL0_WR_BIT_ORDER | SPI_CTRL0_RD_BIT_ORDER)) == 0);
}

void HSPIClass::write16(uint16_t data, bool msb) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // Set to 16Bits transfer
    setDataBits(16);
    if (msb)
    {
        // MSBFIRST Byte first
        SPI(1).W[0] = (data >> 8) | (data << 8);
        SPI(1).CMD |= SPI_CMD_USR;
    }
    else
    {
        // LSBFIRST Byte first
        SPI(1).W[0] = data;
        SPI(1).CMD |= SPI_CMD_USR;
    }
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
}

void HSPIClass::write32(uint32_t data) {
    write32(data, (SPI(1).CTRL0 & (SPI_CTRL0_WR_BIT_ORDER | SPI_CTRL0_RD_BIT_ORDER)) == 0);
}

void HSPIClass::write32(uint32_t data, bool msb) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // Set to 32Bits transfer
    setDataBits(32);
    if(msb) {
        union {
                uint32_t l;
                uint8_t b[4];
        } data_;
        data_.l = data;
        // MSBFIRST Byte first
        SPI(1).W[0] = (data_.b[3] | (data_.b[2] << 8) | (data_.b[1] << 16) | (data_.b[0] << 24));
        SPI(1).CMD |= SPI_CMD_USR;
    } else {
        // LSBFIRST Byte first
        SPI(1).W[0] = data;
        SPI(1).CMD |= SPI_CMD_USR;
    }
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
}

#if 0
void HSPIClass::writeDword(uint32_t data)
{
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // Set to 32Bits transfer
    setDataBits(32);
	// LSBFIRST Byte first
	SPI(1).W[0] = data;
	SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
}
#endif

/**
 * Note:
 *  data need to be aligned to 32Bit
 *  or you get an Fatal exception (9)
 * @param data uint8_t *
 * @param size uint32_t
 */
void HSPIClass::writeBytes(const uint8_t * data, uint32_t size) {
    while(size) {
        if(size > 64) {
            writeBytes_(data, 64);
            size -= 64;
            data += 64;
        } else {
            writeBytes_(data, size);
            size = 0;
        }
    }
}

/**
 * @param data uint32_t *
 * @param size uint32_t
 */
void HSPIClass::writeDwords(const uint32_t * data, uint32_t size) {
    while(size != 0) {
        if(size > 16) {
            writeDwords_(data, 16);
            size -= 16;
            data += 16;
        } else {
            writeDwords_(data, size);
            size = 0;
        }
    }
}

void HSPIClass::writeBytes_(const uint8_t * data, uint8_t size) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // Set Bits to transfer
    setDataBits(size * 8);

    volatile uint32_t * fifoPtr = &SPI(1).W[0];
    uint32_t * dataPtr = (uint32_t*) data;
    uint8_t dataSize = ((size + 3) / 4);

    while(dataSize--) {
        *fifoPtr = *dataPtr;
        dataPtr++;
        fifoPtr++;
    }

	SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
}

void HSPIClass::writeDwords_(const uint32_t * data, uint8_t size) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}

    // Set Bits to transfer
    setDataBits(size * 32);

    volatile uint32_t * fifoPtr = &SPI(1).W[0];
 
    while(size != 0) {
        *fifoPtr++ = *data++;
        size--;
    }
	SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
}

/**
 * Note:
 *  data need to be aligned to 32Bit
 *  or you get an Fatal exception (9)
 * @param data uint8_t *
 * @param size uint8_t  max for size is 64Byte
 * @param repeat uint32_t
 */
void HSPIClass::writePattern(const uint8_t * data, uint8_t size, uint32_t repeat) {
    if(size > 64) return; //max Hardware FIFO

    uint32_t byte = (size * repeat);
    uint8_t r = (64 / size);

    while(byte) {
        if(byte > 64) {
            writePattern_(data, size, r);
            byte -= 64;
        } else {
            writePattern_(data, size, (byte / size));
            byte = 0;
        }
    }
}

void HSPIClass::writePattern_(const uint8_t * data, uint8_t size, uint8_t repeat) {
    uint8_t bytes = (size * repeat);
    uint8_t buffer[64];
    uint8_t * bufferPtr = &buffer[0];
    const uint8_t * dataPtr;
    uint8_t dataSize = bytes;
    for(uint8_t i = 0; i < repeat; i++) {
        dataSize = size;
        dataPtr = data;
        while(dataSize--) {
            *bufferPtr = *dataPtr;
            dataPtr++;
            bufferPtr++;
        }
    }

    writeBytes(&buffer[0], bytes);
}

/**
 * Note:
 *  in and out need to be aligned to 32Bit
 *  or you get an Fatal exception (9)
 * @param out uint8_t *
 * @param in  uint8_t *
 * @param size uint32_t
 */
void HSPIClass::transferBytes(const uint8_t * out, uint8_t * in, uint32_t size) {
    while(size) {
        if(size > 64) {
            transferBytes_(out, in, 64);
            size -= 64;
            if(out) out += 64;
            if(in) in += 64;
        } else {
            transferBytes_(out, in, size);
            size = 0;
        }
    }
}

/**
 * @param out uint32_t *
 * @param in  uint32_t *
 * @param size uint32_t
 */
void HSPIClass::transferDwords(const uint32_t * out, uint32_t * in, uint32_t size) {
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

void HSPIClass::transferBytes_(const uint8_t * out, uint8_t * in, uint8_t size) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}
    // Set in/out Bits to transfer

    setDataBits(size * 8);

    volatile uint32_t * fifoPtr = &SPI(1).W[0];
    uint8_t dataSize = ((size + 3) / 4);

    if(out) {
        uint32_t * dataPtr = (uint32_t*) out;
        while(dataSize--) {
            *fifoPtr = *dataPtr;
            dataPtr++;
            fifoPtr++;
        }
    } else {
        // no out data only read fill with dummy data!
        while(dataSize--) {
            *fifoPtr = 0xFFFFFFFF;
            fifoPtr++;
        }
    }

	SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}

    if(in) {
        volatile uint8_t * fifoPtr8 = (volatile uint8_t *) &SPI(1).W[0];
        dataSize = size;
        while(dataSize--) {
            *in = *fifoPtr8;
            in++;
            fifoPtr8++;
        }
    }
}

void HSPIClass::transferDwords_(const uint32_t * out, uint32_t * in, uint8_t size) {
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}

    // Set in/out Bits to transfer
    setDataBits(size * 32);

    volatile uint32_t * fifoPtr = &SPI(1).W[0];
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

	SPI(1).CMD |= SPI_CMD_USR;
    while ((SPI(1).CMD & SPI_CMD_USR) != 0) {}

    if (in != nullptr) {
        volatile uint32_t * fifoPtrRd = &SPI(1).W[0];
        while(size != 0) {
            *in++ = *fifoPtrRd++;
            size--;
        }
    }
}


