/*
  t4_spi.h - SPI support for Trinamic plugin

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

#ifndef _GRBL_SPI_H_
#define _GRBL_SPI_H_

void t4_spi_init (void);
uint32_t spi_set_speed (uint32_t prescaler);
uint8_t spi_get_byte (void);
uint8_t spi_put_byte (uint8_t byte);
void spi_write (uint8_t *data, uint16_t len);
void spi_read (uint8_t *data, uint16_t len);

#endif
