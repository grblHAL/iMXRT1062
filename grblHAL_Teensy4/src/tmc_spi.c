/*
  tmc_spi.c - driver code for iMXRT1062 ARM processors

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "driver.h"

#if TRINAMIC_SPI_ENABLE

#include "t4_spi.h"
#include "trinamic/common.h"

#define TMC_SPI_FREQ 8000000

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_CS_SINGLE

#if TRINAMIC_SPI_ENABLE & TRINAMIC_SPI_20BIT
#error "20 bit Trinamic SPI datagrams not yet supported!"
#endif

static struct {
    gpio_t port;
} cs;

static uint_fast8_t n_motors;
static TMC_spi_datagram_t datagram[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    static TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(TMC_SPI_FREQ);
#endif

    datagram[driver.seq].addr.value = reg->addr.value;
    datagram[driver.seq].addr.write = 0;

    DIGITAL_OUT(cs.port, 0);

    do {
        spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
        spi_put_byte(0);
    } while(idx);

    delayMicroseconds(1);
    DIGITAL_OUT(cs.port, 1);
    delayMicroseconds(1);
    DIGITAL_OUT(cs.port, 0);

    idx = n_motors;
    do {
        res = spi_put_byte(datagram[--idx].addr.value);

        if(idx == driver.seq) {
            status = res;
            reg->payload.data[3] = spi_get_byte();
            reg->payload.data[2] = spi_get_byte();
            reg->payload.data[1] = spi_get_byte();
            reg->payload.data[0] = spi_get_byte();
        } else {
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
            spi_get_byte();
        }
    } while(idx);

    delayMicroseconds(1);
    DIGITAL_OUT(cs.port, 1);
    delayMicroseconds(1);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    TMC_spi_status_t status = 0;

    uint8_t res;
    uint_fast8_t idx = n_motors;
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(TMC_SPI_FREQ);
#endif

    memcpy(&datagram[driver.seq], reg, sizeof(TMC_spi_datagram_t));
    datagram[driver.seq].addr.write = 1;

    DIGITAL_OUT(cs.port, 0);

    do {
        res = spi_put_byte(datagram[--idx].addr.value);
        spi_put_byte(datagram[idx].payload.data[3]);
        spi_put_byte(datagram[idx].payload.data[2]);
        spi_put_byte(datagram[idx].payload.data[1]);
        spi_put_byte(datagram[idx].payload.data[0]);

        if(idx == driver.seq) {
            status = res;
            datagram[idx].addr.idx = 0; // TMC_SPI_STATUS_REG;
            datagram[idx].addr.write = 0;
        }
    } while(idx);

    delayMicroseconds(1);
    DIGITAL_OUT(cs.port, 1);
    delayMicroseconds(1);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    if(gpio->function == Output_MotorChipSelect) {
        cs.port.reg = (gpio_reg_t *)digital_pin_to_info_PGM[gpio->pin].reg;
        cs.port.bit = digital_pin_to_info_PGM[gpio->pin].mask;
    }
}

static void if_init (uint8_t motors, axes_signals_t axisflags)
{
    n_motors = motors;
    hal.enumerate_pins(true, add_cs_pin, NULL);
}

void tmc_spi_init (void)
{
    trinamic_driver_if_t driver = {
        .on_drivers_init = if_init
    };

    t4_spi_init();

    uint_fast8_t idx = TMC_N_MOTORS_MAX;
    do {
        datagram[--idx].addr.idx = 0; //TMC_SPI_STATUS_REG;
    } while(idx);

    trinamic_if_init(&driver);
}

#else // separate CS pins

static struct {
    gpio_t port;
} cs[TMC_N_MOTORS_MAX];

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(TMC_SPI_FREQ);
#endif

    DIGITAL_OUT(cs[driver.id].port, 0);

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    DIGITAL_OUT(cs[driver.id].port, 1);
    delayMicroseconds(1);
    DIGITAL_OUT(cs[driver.id].port, 0);

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    DIGITAL_OUT(cs[driver.id].port, 1);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(TMC_SPI_FREQ);
#endif

    DIGITAL_OUT(cs[driver.id].port, 0);

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    DIGITAL_OUT(cs[driver.id].port, 1);

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

TMC_spi20_datagram_t tmc_spi20_write (trinamic_motor_t driver, TMC_spi20_datagram_t *datagram)
{
    TMC_spi20_datagram_t status = {0};
#ifndef TRINAMIC_SPI_PORT
    uint32_t f_spi = spi_set_speed(TMC_SPI_FREQ);
#endif

    DIGITAL_OUT(cs[driver.id].port, 0);

    status.data[2] = spi_put_byte(datagram->data[2]);
    status.data[1] = spi_put_byte(datagram->data[1]);
    status.data[0] = spi_put_byte(datagram->data[0]);

    DIGITAL_OUT(cs[driver.id].port, 1);

    status.value >>= 4;

#ifndef TRINAMIC_SPI_PORT
    spi_set_speed(f_spi);
#endif

    return status;
}

static void add_cs_pin (xbar_t *gpio, void *data)
{
    gpio_t *port = NULL;

    if(gpio->group == PinGroup_MotorChipSelect)
      switch (gpio->function) {

        case Output_MotorChipSelectX:
            port = &cs[X_AXIS].port;
            break;

        case Output_MotorChipSelectY:
            port = &cs[Y_AXIS].port;
            break;

        case Output_MotorChipSelectZ:
            port = &cs[Z_AXIS].port;
            break;

        case Output_MotorChipSelectM3:
            port = &cs[3].port;
            break;

        case Output_MotorChipSelectM4:
            port = &cs[4].port;
            break;

        case Output_MotorChipSelectM5:
            port = &cs[5].port;
            break;

        default:
            break;
    }

    if(port)
        memcpy(port, gpio->port, sizeof(gpio_t));
}

static void if_init (uint8_t motors, axes_signals_t enabled)
{
    UNUSED(motors);

    hal.enumerate_pins(true, add_cs_pin, NULL);
}

void tmc_spi_init (void)
{
    static trinamic_driver_if_t driver_if = {.on_drivers_init = if_init};

    t4_spi_init();

    trinamic_if_init(&driver_if);
}
#endif
#endif // TRINAMIC_SPI_ENABLE
