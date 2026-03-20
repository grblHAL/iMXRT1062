/*
  thcad2.c - analog input from frequency, for Mesa THCAD2 converter and iMXRT1062 ARM processor

  Part of grblHAL

  Copyright (c) 2025-2026 Terje Io

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

#include "Arduino.h"
#include "DMAChannel.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "driver.h"

#if THCAD2_ENABLE

#include "grbl/plugins.h"
#include "grbl/ioports.h"
#include "grbl/task.h"

#if !defined(THCAD2_PIN) || !((THCAD2_PIN == 14))
#error "THCAD2: no port/pin or unsupported port/pin"
#endif

static enumerate_pins_ptr on_enumerate_pins;
static io_ports_data_t analog = {};

DMAMEM __attribute__((aligned(32))) volatile uint32_t csr_stopval;

uint counter_slice, gate_slice;
uint32_t timers_enable;
static DMAChannel *dma = NULL;

static xbar_t thcad2 = {
    .ports_id = NULL,
    .id = 0,
    .function = Input_Analog_Aux0,
    .group = PinGroup_AuxInputAnalog,
    .port = &csr_stopval,
    .description = NULL,
    .pin = THCAD2_PIN,
};

static float thcad2_in_state (xbar_t *input)
{
    return (float)csr_stopval;
}

static int32_t thcad2_wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return port < analog.in.n_ports ? (int32_t)csr_stopval : -1;
}

static bool set_pin_function (xbar_t *input, pin_function_t function)
{
    if(input->id == thcad2.id)
        thcad2.function = function;

    return input->id == thcad2.id;
}

static xbar_t *thcad2_get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(dir == Port_Input && port < analog.in.n_ports) {
        pin.get_value = thcad2_in_state;
        pin.set_function = set_pin_function;
        info = &pin;
    }

    return info;
}

static void thcad2_set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        thcad2.description = description;
}

static void onEnumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {};

    on_enumerate_pins(low_level, pin_info, data);

    memcpy(&pin, &thcad2, sizeof(xbar_t));

    if(!low_level)
        pin.port = (void *)"THCAD2:";

    pin_info(&pin, data);
}

static void get_next_port (xbar_t *pin, void *fn)
{
    if(pin->group == PinGroup_AuxInputAnalog)
        *(pin_function_t *)fn = (pin_function_t)max(*(pin_function_t *)fn, pin->function + 1);
}

void dma_counter_capture (void)
{
    GPT2_CR &= ~GPT_CR_EN;  // Reset counter
    GPT2_CR |= GPT_CR_EN;

    dma->clearInterrupt();
    arm_dcache_delete((void *)&csr_stopval, 4);
}

FLASHMEM void thcad2_init (void)
{
    static const periph_pin_t ssp = {
        .function = Input_SpindlePulse,
        .group = PinGroup_AuxInputAnalog,
        .port = NULL,
        .pin = THCAD2_PIN,
        .mode = { .mask = PINMODE_NONE }
    };

    io_analog_t ports = {
        .ports = &analog,
        .analog_out = NULL,
        .wait_on_input = thcad2_wait_on_input,
        .set_pin_description = thcad2_set_pin_description,
        .get_pin_info = thcad2_get_pin_info
    };

    if(!(dma = new DMAChannel))
        return; // unable to allocate DMA channel

    if(dma->channel == 0 && !(dma = new DMAChannel))
        return; // unable to allocate DMA channel

    thcad2.cap.input = On;
    thcad2.cap.analog = On;
    thcad2.cap.external = On;
    thcad2.cap.claimable = On;
    thcad2.cap.resolution = Resolution_12bit;
    thcad2.mode.analog = On;

    hal.enumerate_pins(false, get_next_port, &thcad2.function);

    analog.in.n_ports = 1;

    if(ioports_add_analog(&ports)) {

        CCM_CMEOR |= CCM_CMEOR_MOD_EN_OV_GPT;
        CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);
        CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);

        // Free running timer
        PIT_LDVAL1 = (hal.f_step_timer / 200) - 1; // 10 ms sampling rate (for some reason this has to run at 5 ms).
        PIT_TFLG1 |= PIT_TFLG_TIF;

        IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
        IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = IOMUXC_PAD_DSE(0) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
        IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

        // Pulse counter
        GPT2_CR = 0;
        GPT2_CR |= GPT_CR_SWR;
        while(GPT2_CR & GPT_CR_SWR);
        GPT2_PR = 0;
        GPT2_CR = GPT_CR_CLKSRC(3);
        GPT2_CR |= GPT_CR_ENMOD;
        GPT2_CR |= GPT_CR_FRR|GPT_CR_EN;

        volatile uint16_t *r = (uint16_t *)IMXRT_XBARA1_ADDRESS;
        volatile uint32_t *mux = &DMAMUX_CHCFG0 + dma->channel;

        *r = (*r & 0x00FF) | (XBARA1_IN_PIT_TRIGGER1<<8);

        XBARA1_CTRL0 |= (XBARA_CTRL_DEN1|XBARA_CTRL_EDGE1(1));

        dma->source((volatile uint32_t &)GPT2_CNT);
        dma->destination((volatile uint32_t &)csr_stopval);
        dma->transferSize(sizeof(uint32_t));
        dma->transferCount(1);
        dma->interruptAtCompletion();
        dma->attachInterrupt(dma_counter_capture);
        dma->triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_1); // DMA_CH_MUX_REQ31

        uint32_t source = *mux &0x7f;

        *mux = 0;
        *mux = DMAMUX_CHCFG_TRIG | source;
        *mux |= DMAMUX_CHCFG_ENBL;

        dma->enable();

        PIT_TCTRL1 |= PIT_TCTRL_TEN;

        on_enumerate_pins = hal.enumerate_pins;
        hal.enumerate_pins = onEnumeratePins;

        hal.periph_port.register_pin(&ssp);
    }
}

#endif // THCAD2_ENABLE

#ifdef __cplusplus
}
#endif
