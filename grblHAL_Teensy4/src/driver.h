/*
  driver.h - driver code for IMXRT1062 processor (on Teensy 4.x board)

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

#ifndef MCP3221_ENABLE
#define MCP3221_ENABLE 0
#else
#define I2C_ENABLE 1
#endif

#if MODBUS_ENABLE & 0b100 // Modbus TCP
#undef ETHERNET_ENABLE
#define ETHERNET_ENABLE 1
#endif

#include "grbl/driver_opts.h"

#define DIGITAL_IN(gpio) (!!(gpio.reg->DR & gpio.bit))
#define DIGITAL_OUT(gpio, on) { if(on) gpio.reg->DR_SET = gpio.bit; else gpio.reg->DR_CLEAR = gpio.bit; }

#define timer(t) timerN(t)
#define timerN(t) TMR ## t
#define timerINT(t) timerint(t)
#define timerint(t) IRQ_QTIMER ## t
#define timerENABLE(t) timerena(t)
#define timerena(t) TMR ## t ## _ENBL
#define timerCTRL(t, n) timerctrl(t, n)
#define timerctrl(t, n) TMR ## t ## _CTRL ## n
#define timerCMPLD1(t, n) timercmpld1(t, n)
#define timercmpld1(t, n) TMR ## t ## _CMPLD1 ## n
#define timerCOMP1(t, n) timercomp1(t, n)
#define timercomp1(t, n) TMR ## t ##  _COMP1 ## n
#define timerCOMP2(t, n) timercomp2(t, n)
#define timercomp2(t, n) TMR ## t ##  _COMP2 ## n
#define timerSCTRL(t, n) timersctrl(t, n)
#define timersctrl(t, n) (TMR ## t ## _SCTRL ## n)
#define timerCSCTRL(t, n) timercsctrl(t, n)
#define timercsctrl(t, n) (TMR ## t ## _CSCTRL ## n)
#define timerLOAD(t, n) timerload(t, n)
#define timerload(t, n) TMR ## t ## _LOAD ## n

#define PULSE_TIMER_N               4
#define PULSE_TIMER                 timer(PULSE_TIMER_N)
#define PULSE_TIMER_ENABLE          timerENABLE(PULSE_TIMER_N)
#define PULSE_TIMER_CTRL            timerCTRL(PULSE_TIMER_N, 0)
#define PULSE_TIMER_CSCTRL          timerCSCTRL(PULSE_TIMER_N, 0)
#define PULSE_TIMER_COMP1           timerCOMP1(PULSE_TIMER_N, 0)
#define PULSE_TIMER_LOAD            timerLOAD(PULSE_TIMER_N, 0)
#define PULSE_TIMER_IRQ             timerINT(PULSE_TIMER_N)

// Other timer assignments (for reference)

//#define STEPPER_TIMER     PIT0 (32 bit)

// Timers used for spindle encoder if spindle sync is enabled:
//#define RPM_TIMER         GPT1
//#define RPM_COUNTER       GPT2

#if WEBUI_ENABLE
  #ifdef WEBUI_INFLASH
  #undef WEBUI_INFLASH
  #endif
  #ifdef LITTLEFS_ENABLE
  #undef LITTLEFS_ENABLE
  #endif
#define WEBUI_INFLASH   1
#define LITTLEFS_ENABLE 1
#endif

#ifdef BOARD_CNC_BOOSTERPACK
  #include "boards/cnc_boosterpack_map.h"
#elif defined(BOARD_T40X101)
  #include "boards/T40X101_map.h"
#elif defined(BOARD_T41U5XBB)
  #include "boards/T41U5XBB_map.h"
#elif defined(BOARD_T41U5XBB_SS)
  #include "boards/T41U5XBB_ss_map.h"
#elif defined(BOARD_T41BB5X_PRO)
  #include "boards/T41BB5X_Pro_map.h"
#elif defined(BOARD_GRBLHAL2000)
  #include "boards/GRBLHAL2000_map.h"
#elif defined(BOARD_E5XMCS_T41)
  #include "boards/E5XMCS_T41_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "boards/my_machine_map.h"
#else // default board
#include "boards/generic_map.h"
#endif

#if SPINDLE_PWM_PIN && !(SPINDLE_PWM_PIN == 12 || SPINDLE_PWM_PIN == 13)
  #error "SPINDLE_PWM_PIN can only be routed to pin 12 or 13!"
#endif

#if STEP_INJECT_ENABLE && PPI_ENABLE
#error "Plasma and PPI mode cannot be enabled at the same time!"
#endif

#if STEP_INJECT_ENABLE

#if SPINDLE_PWM_PIN == 12
#define PULSE2_TIMER_N              2
#else
#define PULSE2_TIMER_N              1
#endif
#define PULSE2_TIMER                timer(PULSE2_TIMER_N)
#define PULSE2_TIMER_ENABLE         timerENABLE(PULSE2_TIMER_N)
#define PULSE2_TIMER_CTRL           timerCTRL(PULSE2_TIMER_N, 0)
#define PULSE2_TIMER_CSCTRL         timerCSCTRL(PULSE2_TIMER_N, 0)
#define PULSE2_TIMER_COMP1          timerCOMP1(PULSE2_TIMER_N, 0)
#define PULSE2_TIMER_LOAD           timerLOAD(PULSE2_TIMER_N, 0)
#define PULSE2_TIMER_IRQ            timerINT(PULSE2_TIMER_N)

#endif

#if PPI_ENABLE

#if SPINDLE_PWM_PIN == 12
#define PPI_TIMER_N                 2
#else
#define PPI_TIMER_N                 1
#endif
#define PPI_TIMER                   timer(PPI_TIMER_N)
#define PPI_TIMER_ENABLE            timerENABLE(PPI_TIMER_N)
#define PPI_TIMER_CTRL              timerCTRL(PPI_TIMER_N, 0)
#define PPI_TIMER_CSCTRL            timerCSCTRL(PPI_TIMER_N, 0)
#define PPI_TIMER_COMP1             timerCOMP1(PPI_TIMER_N, 0)
#define PPI_TIMER_LOAD              timerLOAD(PPI_TIMER_N, 0)
#define PPI_TIMER_IRQ               timerINT(PPI_TIMER_N)

#endif

#if SPINDLE_PWM_PIN == 12
#define SPINDLE_PWM_TIMER_N         1
#define SPINDLE_PWM_TIMER_C         1
#else
#define SPINDLE_PWM_TIMER_N         2
#define SPINDLE_PWM_TIMER_C         0
#endif
#define SPINDLE_PWM_TIMER           timer(SPINDLE_PWM_TIMER_N)
#define SPINDLE_PWM_TIMER_ENABLE    timerENABLE(SPINDLE_PWM_TIMER_N)
#define SPINDLE_PWM_TIMER_CTRL      timerCTRL(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_C)
#define SPINDLE_PWM_TIMER_SCTRL     timerSCTRL(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_C)
#define SPINDLE_PWM_TIMER_COMP1     timerCOMP1(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_C)
#define SPINDLE_PWM_TIMER_COMP2     timerCOMP2(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_C)
#define SPINDLE_PWM_TIMER_CMPLD1    timerCMPLD1(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_C)
#define SPINDLE_PWM_TIMER_LOAD      timerLOAD(SPINDLE_PWM_TIMER_N, SPINDLE_PWM_TIMER_C)

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

#if USB_SERIAL_CDC
#define SP0 1
#else
#define SP0 0
#endif

#if MODBUS_ENABLE & MODBUS_RTU_ENABLED
#define MODBUS_TEST 1
#else
#define MODBUS_TEST 0
#endif

#if MPG_ENABLE
#define MPG_TEST 1
#else
#define MPG_TEST 0
#endif

#if KEYPAD_ENABLE == 2 && MPG_ENABLE == 0
#define KEYPAD_TEST 1
#else
#define KEYPAD_TEST 0
#endif

#if USB_SERIAL_CDC == 0
#if (MODBUS_TEST + KEYPAD_TEST + MPG_TEST + BLUETOOTH_ENABLE)
#error "Options that uses the UART serial port can only be enabled with USB serial CDC!"
#endif
#elif (MODBUS_TEST + KEYPAD_TEST + MPG_TEST + (BLUETOOTH_ENABLE ? 1 : 0)) > SP0
#error "Only one option that uses the UART serial port can be enabled!"
#endif

#undef SP0
#undef MODBUS_TEST
#undef KEYPAD_TEST
#undef MPG_TEST

#if MPG_ENABLE == 1 && !defined(MPG_MODE_PIN)
#error "MPG_MODE_PIN must be defined!"
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
#endif

#if KEYPAD_ENABLE == 1 && !defined(I2C_STROBE_PIN)
#error Keypad plugin not supported!
#elif I2C_STROBE_ENABLE && !defined(I2C_STROBE_PIN)
#error I2C strobe not supported!
#endif

#ifndef I2C_PORT
  #if EEPROM_ENABLE
  #error "EEPROM_ENABLE requires I2C_PORT to be defined!"
  #endif
  #if I2C_STROBE_ENABLE
  #error "KEYPAD_ENABLE requires I2C_PORT to be defined!"
  #endif
  #if MCP3221_ENABLE
  #error "MCP3221_ENABLE requires I2C_PORT to be defined!"
  #endif
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
    pin_cap_t cap;
    pin_mode_t mode;
    uint8_t pin;
    uint32_t bit;
    pin_group_t group;
    gpio_t *port;
    gpio_t gpio; // doubled up for now for speed...
    uint8_t offset;
    uint8_t user_port;
    volatile bool active;
    ioport_interrupt_callback_ptr interrupt_callback;
    const char *description;
} input_signal_t;

typedef struct {
    pin_function_t id;
    gpio_t *port;
    uint8_t pin;
    pin_group_t group;
    pin_mode_t mode;
    const char *description;
    uint8_t pwm_idx;
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

void ioports_init (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs);
void ioports_event (input_signal_t *input);

#endif // __DRIVER_H__
