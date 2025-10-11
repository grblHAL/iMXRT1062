/*
  t4_spi.c - SPI support for Trinamic plugin

  Part of grblHAL driver for iMXRT1062

  Copyright (c) 2020-2025 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if SPI_ENABLE

#include <spi.h>
#include "Arduino.h"

#ifndef SPI_FREQ
#define SPI_FREQ 8000000
#endif

static SPISettings cfg(SPI_FREQ, MSBFIRST, SPI_MODE3);

#ifdef __cplusplus
extern "C" {
#endif

void t4_spi_init (void)
{
    static bool init = false;

    if(!init) {

        init = true;

        static const periph_pin_t sck = {
            .function = Output_SPICLK,
            .group = PinGroup_SPI,
            .port = NULL,
            .pin = 13,
            .mode = { .mask = PINMODE_OUTPUT }
        };

        static const periph_pin_t sdo = {
            .function = Input_MISO,
            .group = PinGroup_SPI,
            .port = NULL,
            .pin = 12,
            .mode = { .mask = PINMODE_NONE }
        };

        static const periph_pin_t sdi = {
            .function = Output_MOSI,
            .group = PinGroup_SPI,
            .port = NULL,
            .pin = 11,
            .mode = { .mask = PINMODE_NONE }
        };

        hal.periph_port.register_pin(&sck);
        hal.periph_port.register_pin(&sdo);
        hal.periph_port.register_pin(&sdi);

        SPI.begin();
    }
}

uint32_t spi_set_speed (uint32_t f_hz)
{
    static uint32_t cur = SPI_FREQ;

    if(f_hz != cur) {
//        SPI.setFrequency(f_hz);
        f_hz = cur;
    }

    return cur;
}

uint8_t spi_get_byte (void)
{
    SPI.beginTransaction(cfg);
    uint8_t data = SPI.transfer(0xFF);
    SPI.endTransaction();

    return data;
}

uint8_t spi_put_byte (uint8_t byte)
{
    SPI.beginTransaction(cfg);

    byte = SPI.transfer(byte);

    SPI.endTransaction();

    return byte;
}

void spi_write (uint8_t *data, uint16_t len)
{
    if(len) {

        SPI.beginTransaction(cfg);

        do {
            SPI.transfer(*data++);
        } while(--len);

        SPI.endTransaction();
    }
}

void spi_read (uint8_t *data, uint16_t len)
{
    if(len) {

        SPI.beginTransaction(cfg);

        do {
            *data++ = SPI.transfer(0);
        } while(--len);

        SPI.endTransaction();
    }
}

#ifdef __cplusplus
}
#endif

#endif // SPI_ENABLE
