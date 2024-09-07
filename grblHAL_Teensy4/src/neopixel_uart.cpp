/*
  neopixel_uart.c - UART support for Neopixels

  Part of grblHAL driver for iMXRT1062

  Some parts Copyright (c) 2024 Terje Io
  Some parts parts derived from WS2812Serial - Non-blocking WS2812 LED Display Library:
    https://github.com/PaulStoffregen/WS2812Serial
    Copyright (c) 2017 Paul Stoffregen, PJRC.COM, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "driver.h"

#ifdef NEOPIXEL_UART_PIN

#include "Arduino.h"
#include "DMAChannel.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NEOPIXELS_NUM
#define NEOPIXELS_NUM 1
#endif

static DMAChannel *dma = NULL;
static uint32_t prior_micros = 0;
static IMXRT_LPUART_t *uart = NULL;
static uint8_t *frameBuffer = NULL;
static neopixel_cfg_t neopixel = {
    .num_leds = 0,
    .num_bytes = 0,
    .leds = NULL,
    .intensity = 255
};
static settings_changed_ptr settings_changed;

void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(neopixel.leds == NULL || hal.rgb0.num_devices != settings->rgb_strip0_length) {

        if(settings->rgb_strip0_length == 0)
            settings->rgb_strip0_length = hal.rgb0.num_devices;
        else
            hal.rgb0.num_devices = settings->rgb_strip0_length;

        if(neopixel.leds) {
            free(neopixel.leds);
            neopixel.leds = NULL;
        }

        if(hal.rgb0.num_devices) {
            neopixel.num_bytes = hal.rgb0.num_devices * 3;
            if((neopixel.leds = (uint8_t *)calloc(neopixel.num_bytes, sizeof(uint8_t)))) {
                if(!(frameBuffer = (uint8_t *)malloc(hal.rgb0.num_devices * 12))) {
                    hal.rgb0.num_devices = 0;
                    free(neopixel.leds);
                    neopixel.leds = NULL;
                }
            }
        }

        neopixel.num_leds = hal.rgb0.num_devices;
    }

    if(settings_changed)
        settings_changed(settings, changed);
}

static void _write (void)
{
	while((DMA_ERQ & (1 << dma->channel)));

	rgb_color_t color;
	uint32_t microseconds_per_led = 30, bytes_per_led = 12;

	const uint8_t *p = neopixel.leds;
	const uint8_t *end = p + neopixel.num_leds * 3;
	uint8_t *fb = frameBuffer;

	while (p < end) {

		color.G = *p++;
		color.R = *p++;
		color.B = *p++;
		color = rgb_set_intensity(color, neopixel.intensity);

		uint32_t n = (color.G << 16) | (color.R << 8) | color.B;

		const uint8_t *stop = fb + 12;
		do {
			uint8_t x = 0x08;
			if (!(n & 0x00800000)) x |= 0x07;
			if (!(n & 0x00400000)) x |= 0xE0;
			n <<= 2;
			*fb++ = x;
		} while (fb < stop);
	}
	microseconds_per_led = 30;
	bytes_per_led = 12;

	// wait 300us WS2812 reset time
	uint32_t m, min_elapsed = (neopixel.num_leds * microseconds_per_led) + 300;

	while(true) {
		if(((m = micros()) - prior_micros) > min_elapsed)
			break;
	}
	prior_micros = m;

	// start DMA transfer to update LEDs

	// See if we need to muck with DMA cache...
	if((uint32_t)frameBuffer >= 0x20200000u)
		arm_dcache_flush(frameBuffer, neopixel.num_leds * bytes_per_led);

	dma->sourceBuffer(frameBuffer, neopixel.num_leds * bytes_per_led);
	dma->transferCount(neopixel.num_leds * bytes_per_led);
	dma->disableOnCompletion();

	uart->STAT = 0; // try clearing out the status
	dma->enable();
}

void neopixels_write (void)
{
    if(neopixel.num_leds > 1)
		 _write();
}

static void neopixel_out_masked (uint16_t device, rgb_color_t color, rgb_color_mask_t mask)
{
    if(neopixel.num_leds && device < neopixel.num_leds) {

        rgb_1bpp_assign(&neopixel.leds[device * 3], color, mask);

        if(neopixel.num_leds == 1)
            _write();
    }
}

static void neopixel_out (uint16_t device, rgb_color_t color)
{
    static const rgb_color_mask_t mask = { .mask = 0xFF };

    neopixel_out_masked(device, color, mask);
}

static uint8_t neopixels_set_intensity (uint8_t intensity)
{
    uint8_t prev = neopixel.intensity;

    if(neopixel.intensity != intensity) {

        neopixel.intensity = intensity;

        if(neopixel.num_leds)
            _write();
    }

    return prev;
}

void neopixel_init (void)
{
    static bool init = false;

    if(!init) {

        uint32_t hwtrigger;

        switch (NEOPIXEL_UART_PIN) {

          case 1: // Serial1
    #if defined(ARDUINO_TEENSY41)
          case 53:
    #endif
            uart = &IMXRT_LPUART6;
            CCM_CCGR3 |= CCM_CCGR3_LPUART6(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART6_TX;
            break;
          case 8: // Serial2
            uart = &IMXRT_LPUART4;
            CCM_CCGR1 |= CCM_CCGR1_LPUART4(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART4_TX;
            break;
          case 14: // Serial3
            uart = &IMXRT_LPUART2;
            CCM_CCGR0 |= CCM_CCGR0_LPUART2(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART2_TX;
            break;
          case 17: // Serial4
            uart = &IMXRT_LPUART3;
            CCM_CCGR0 |= CCM_CCGR0_LPUART3(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART3_TX;
            break;
          case 20: // Serial5
    #if defined(ARDUINO_TEENSY40)
          case 39: // Serial5 alt
    #elif defined(ARDUINO_TEENSY41)
          case 47:
    #endif
            uart = &IMXRT_LPUART8;
            CCM_CCGR6 |= CCM_CCGR6_LPUART8(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART8_TX;
            break;
          case 24: // Serial6
            uart = &IMXRT_LPUART1;
            CCM_CCGR5 |= CCM_CCGR5_LPUART1(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART1_TX;
            break;
          case 29: // Serial7
            uart = &IMXRT_LPUART7;
            CCM_CCGR5 |= CCM_CCGR5_LPUART7(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART7_TX;
            break;
    #if defined(ARDUINO_TEENSY41)
          case 35:
            uart = &IMXRT_LPUART5;
            CCM_CCGR3 |= CCM_CCGR3_LPUART5(CCM_CCGR_ON);
            hwtrigger = DMAMUX_SOURCE_LPUART5_TX;
            break;
    #endif
          default:
            return; // pin not supported
        }

        if(!(dma = new DMAChannel))
            return; // unable to allocate DMA channel

        // Convert Baud, computed values for 4mhz
        uart->CTRL = 0;
        uart->BAUD = LPUART_BAUD_OSR(5) | LPUART_BAUD_SBR(1) | LPUART_BAUD_TDMAE;  // set baud configure for transfer DMA
        uart->PINCFG = 0;
        uint16_t tx_fifo_size = (((uart->FIFO >> 4) & 0x7) << 2);
        uint8_t tx_water = (tx_fifo_size < 16) ? tx_fifo_size >> 1 : 7;
        uart->WATER = LPUART_WATER_TXWATER(tx_water);
        uart->FIFO |= LPUART_FIFO_TXFE;
        uart->CTRL = (LPUART_CTRL_TE | LPUART_CTRL_TXINV); // enable transmitter and invert

        *(portControlRegister(NEOPIXEL_UART_PIN)) =  IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
#if NEOPIXEL_UART_PIN == 35
        *(portConfigRegister(NEOPIXEL_UART_PIN)) = 1;
#else
        *(portConfigRegister(NEOPIXEL_UART_PIN)) = 2;
#endif

        dma->destination((volatile uint8_t &)uart->DATA);
        dma->triggerAtHardwareEvent(hwtrigger);

        static const periph_pin_t neopix = {
            .function = Output_LED_Adressable,
            .group = PinGroup_UART,
            .port = NULL,
            .pin = NEOPIXEL_UART_PIN,
            .mode = { .mask = PINMODE_OUTPUT },
            .description = "Neopixels"
        };

        hal.periph_port.register_pin(&neopix);

        hal.rgb0.out = neopixel_out;
        hal.rgb0.out_masked = neopixel_out_masked;
        hal.rgb0.set_intensity = neopixels_set_intensity;
        hal.rgb0.write = neopixels_write;
        hal.rgb0.num_devices = NEOPIXELS_NUM;
        hal.rgb1.flags = (rgb_properties_t){ .is_strip = On };
        hal.rgb0.cap.R = hal.rgb0.cap.G = hal.rgb0.cap.B = 255;

        settings_changed = hal.settings_changed;
        hal.settings_changed = onSettingsChanged;

        init = true;
    }
}

#ifdef __cplusplus
}
#endif

#endif // NEOPIXEL_UART_PIN
