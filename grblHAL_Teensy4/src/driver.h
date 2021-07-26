/*
  driver.h - driver code for IMXRT1062 processor (on Teensy 4.x board)

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "imxrt.h"
#include "core_pins.h"
#include "pins_arduino.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/driver_opts.h"

#define DIGITAL_IN(gpio) (!!(gpio.reg->DR & gpio.bit))
#define DIGITAL_OUT(gpio, on) { if(on) gpio.reg->DR_SET = gpio.bit; else gpio.reg->DR_CLEAR = gpio.bit; }

#ifndef MCP3221_ENABLE
#define MCP3221_ENABLE 0
#endif

// Timer assignments (for reference, Arduino libs does not follow the CMSIS style...)

//#define STEPPER_TIMER     PIT0 (32 bit)
//#define PULSE_TIMER       TMR4
//#define SPINDLE_PWM_TIMER TMR1 (pin 12) or TMR2 (pin 3)
//#define DEBOUNCE_TIMER    TMR3
//#define PLASMA_TIMER      TMR2
//#define PPI_TIMER         inverse of SPINDLE_PWM_TIMER

// Timers used for spindle encoder if spindle sync is enabled:
//#define RPM_TIMER         GPT1
//#define RPM_COUNTER       GPT2

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_T40X101)
  #include "T40X101_map.h"
#elif defined(BOARD_T41U5XBB)
  #include "T41U5XBB_map.h"
#elif defined(BOARD_T41U5XBB_SS)
  #include "T41U5XBB_ss_map.h"
#elif defined(BOARD_T41BB5X_PRO)
  #include "T41BB5X_Pro_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else // default board
#include "generic_map.h"
#endif

#if SPINDLEPWMPIN == 12
#define PPI_TIMER       (IMXRT_TMR2)
#define PPI_TIMERIRQ    IRQ_QTIMER2
#else
#define PPI_TIMER       (IMXRT_TMR1)
#define PPI_TIMERIRQ    IRQ_QTIMER1
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 0.2f // microseconds
#endif

#ifndef IOPORTS_ENABLE
#define IOPORTS_ENABLE 0
#endif

#if EEPROM_ENABLE && !defined(EEPROM_IS_FRAM)
#define EEPROM_IS_FRAM  0
#endif

#if TRINAMIC_ENABLE
#include "tmc2130/trinamic.h"
#endif

#if QEI_ENABLE
#include "encoder/encoder.h"
#endif

#if BLUETOOTH_ENABLE && USB_SERIAL_CDC == 0
#error "Bluetooth cannot be used with UART communications enabled!"
#endif

#if SPINDLE_HUANYANG
#if USB_SERIAL_CDC == 0
#error "Huanyang VFD cannot be used with UART communications enabled!"
#endif
#include "spindle/huanyang.h"
#endif

#ifndef VFD_SPINDLE
#define VFD_SPINDLE 0
#endif

#if PLASMA_ENABLE
#include "plasma/thc.h"
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
#endif

#if KEYPAD_ENABLE && !defined(KEYPAD_STROBE_PIN)
#error "KEYPAD_ENABLE requires KEYPAD_STROBE_PIN to be defined!"
#endif

#ifndef I2C_PORT
  #if EEPROM_ENABLE
  #error "EEPROM_ENABLE requires I2C_PORT to be defined!"
  #endif
  #if KEYPAD_ENABLE
  #error "KEYPAD_ENABLE requires I2C_PORT to be defined!"
  #endif
  #if MCP3221_ENABLE
  #error "MCP3221_ENABLE requires I2C_PORT to be defined!"
  #endif
#endif

#if !(SPINDLEPWMPIN == 12 || SPINDLEPWMPIN == 13)
  #error "SPINDLEPWMPIN can only be routed to pin 12 or 13!"
#endif

#if QEI_ENABLE > 1
  #error "Max number of quadrature interfaces is 1!"
#endif

#if QEI_ENABLE > 0 && !(defined(QEI_A_PIN) && defined(QEI_B_PIN))
  #error "QEI_ENABLE requires encoder input pins A and B to be defined!"
#endif

typedef struct {
    volatile uint32_t DR;
    volatile uint32_t GDIR;
    volatile uint32_t PSR;
    volatile uint32_t ICR1;
    volatile uint32_t ICR2;
    volatile uint32_t IMR;
    volatile uint32_t ISR;
    volatile uint32_t EDGE_SEL;
    uint32_t unused[25];
    volatile uint32_t DR_SET;
    volatile uint32_t DR_CLEAR;
    volatile uint32_t DR_TOGGLE;
} gpio_reg_t;

typedef struct {
    gpio_reg_t *reg;
    uint32_t bit;
} gpio_t;

typedef struct {
    pin_function_t id;
    pin_group_t group;
    uint8_t pin;
    gpio_t *port;
    gpio_t gpio; // doubled up for now for speed...
    pin_irq_mode_t irq_mode;
    uint8_t offset;
    volatile bool active;
    volatile bool debounce;
    pin_mode_t cap;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    gpio_t *port;
    uint8_t pin;
    pin_group_t group;
    const char *description;
} output_signal_t;

typedef struct {
    uint8_t n_pins;
    union {
        input_signal_t *inputs;
        output_signal_t *outputs;
    } pins;
} pin_group_pins_t;

// The following struct is pulled from the Teensy Library core, Copyright (c) 2019 PJRC.COM, LLC.

typedef struct {
    const uint8_t pin;              // The pin number
    const uint32_t mux_val;         // Value to set for mux;
    volatile uint32_t *select_reg;  // Which register controls the selection
    const uint32_t select_val;      // Value for that selection
} pin_info_t;

void pinModeOutput (gpio_t *gpio, uint8_t pin);
void pinEnableIRQ (const input_signal_t *signal, pin_irq_mode_t irq_mode);
uint32_t xTaskGetTickCount();

#ifdef HAS_IOPORTS
void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);
#endif

#endif // __DRIVER_H__
