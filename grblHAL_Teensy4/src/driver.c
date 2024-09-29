/*
  driver.c - driver code for IMXRT1062 processor (on Teensy 4.0/4.1 board)

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

// shut up compiler warning...
#pragma GCC diagnostic ignored "-Wunused-function"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AUX_DEVICES // until all drivers are converted?

#include "uart.h"
#include "driver.h"
#include "grbl/protocol.h"
#include "grbl/machine_limits.h"
#include "grbl/state_machine.h"
#include "grbl/pin_bits_masks.h"

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#else
#include "avr/eeprom.h"
#endif

#if IOPORTS_ENABLE
#include "ioports.h"
#endif

#if SDCARD_ENABLE
#include "uSDFS.h"
#include "sdcard/sdcard.h"
#endif

#if LITTLEFS_ENABLE
#include "littlefs_hal.h"
#include "sdcard/fs_littlefs.h"
#endif

#if PPI_ENABLE
#include "laser/ppi.h"
static void ppi_timeout_isr (void);
#endif

#if ETHERNET_ENABLE
  #include "enet.h"
#endif

#if USB_SERIAL_CDC == 1
#include "usb_serial_ard.h"
#elif USB_SERIAL_CDC == 2
#include "usb_serial_pjrc.h"
#endif

#define F_BUS_MHZ (F_BUS_ACTUAL / 1000000)

#include "grbl/motor_pins.h"

#if QEI_ENABLE

#define QEI_DEBOUNCE 3
#define QEI_VELOCITY_TIMEOUT 100

typedef union {
    uint_fast8_t pins;
    struct {
        uint_fast8_t a :1,
                     b :1;
    };
} qei_state_t;

typedef struct {
    encoder_t encoder;
    int32_t count;
    int32_t vel_count;
    uint_fast16_t state;
    volatile uint32_t dbl_click_timeout;
    volatile uint32_t vel_timeout;
    uint32_t vel_timestamp;
} qei_t;

static qei_t qei = {0};

#endif

// Standard inputs
static gpio_t Reset, FeedHold, CycleStart, LimitX, LimitY, LimitZ;

// Standard outputs
static gpio_t Flood, stepX, stepY, stepZ, dirX, dirY, dirZ;
#ifdef COOLANT_MIST_PIN
static gpio_t Mist;
#endif

#if DRIVER_SPINDLE_ENABLE
static spindle_id_t spindle_id = -1;
static gpio_t spindleEnable, spindleDir;
#endif
#if DRIVER_SPINDLE_PWM_ENABLE
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;
#endif

#if SPINDLE_ENCODER_ENABLE

#include "grbl/spindle_sync.h"

static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static void spindle_pulse_isr (void);
static on_spindle_programmed_ptr on_spindle_programmed = NULL;

#if SPINDLE_SYNC_ENABLE
static spindle_sync_t spindle_tracker;
static volatile bool spindleLock = false;
#endif

#endif // SPINDLE_ENCODER_ENABLE

#if SPINDLE_SYNC_ENABLE
static void stepperPulseStartSynchronized (stepper_t *stepper);
#endif

// Optional I/O

#ifndef AUX_DEVICES

#if PROBE_ENABLE
static Probe;
#endif
#if I2C_STROBE_ENABLE
static gpio_t KeypadStrobe;
#endif
#if MPG_ENABLE == 1
static gpio_t ModeSelect;
static input_signal_t *mpg_pin = NULL;
#endif
#if QEI_ENABLE && defined(QEI_SELECT_PIN)
#define QEI_SELECT_ENABLED 1
static gpio_t QEI_Select;
#endif

#endif // AUX_DEVICES

#ifdef A_AXIS
static gpio_t stepA, dirA, LimitA;
#ifdef A_ENABLE_PIN
static gpio_t enableA;
#endif
#endif
#ifdef B_AXIS
static gpio_t stepB, dirB, LimitB;
#ifdef B_ENABLE_PIN
static gpio_t enableB;
#endif
#endif
#ifdef C_AXIS
static gpio_t stepC, dirC;
#ifdef C_ENABLE_PIN
static gpio_t enableC;
#endif
#ifdef C_LIMIT_PIN
static gpio_t LimitC;
#endif
#endif
#ifdef STEPPERS_ENABLE_PIN
static gpio_t steppersEnable;
#endif
#ifdef X_ENABLE_PIN
static gpio_t enableX;
#endif
#ifdef Y_ENABLE_PIN
static gpio_t enableY;
#endif
#ifdef Z_ENABLE_PIN
static gpio_t enableZ;
#endif

#if QEI_ENABLE
static bool qei_enable = false;
static gpio_t QEI_A, QEI_B;
 #ifdef QEI_INDEX_PIN
  #define QEI_INDEX_ENABLED 1
  static gpio_t QEI_Index;
 #endif
#endif

#ifdef X2_STEP_PIN
  static gpio_t stepX2;
#endif
#ifdef X2_DIRECTION_PIN
  static gpio_t dirX2;
#endif
#ifdef X2_ENABLE_PIN
  static gpio_t enableX2;
#endif
#ifdef X2_LIMIT_PIN
  static gpio_t LimitX2;
#endif
#ifdef X_LIMIT_PIN_MAX
  static gpio_t LimitXMax;
#endif

#ifdef Y2_STEP_PIN
  static gpio_t stepY2;
#endif
#ifdef Y2_DIRECTION_PIN
  static gpio_t dirY2;
#endif
#ifdef Y2_ENABLE_PIN
  static gpio_t enableY2;
#endif
#ifdef Y2_LIMIT_PIN
  static gpio_t LimitY2;
#endif
#ifdef Y_LIMIT_PIN_MAX
  static gpio_t LimitYMax;
#endif

#ifdef Z2_STEP_PIN
  static gpio_t stepZ2;
#endif
#ifdef Z2_DIRECTION_PIN
  static gpio_t dirZ2;
#endif
#ifdef Z2_ENABLE_PIN
  static gpio_t enableZ2;
#endif
#ifdef Z2_LIMIT_PIN
  static gpio_t LimitZ2;
#endif
#ifdef Z_LIMIT_PIN_MAX
  static gpio_t LimitZMax;
#endif

#ifdef SPINDLE_INDEX_PIN
  static gpio_t SpindleIndex;
#endif

#ifdef AUXINPUT0_PIN
  static gpio_t AuxIn0;
#endif
#ifdef AUXINPUT1_PIN
  static gpio_t AuxIn1;
#endif
#ifdef AUXINPUT2_PIN
  static gpio_t AuxIn2;
#endif
#ifdef AUXINPUT3_PIN
  static gpio_t AuxIn3;
#endif
#ifdef AUXINPUT4_PIN
  static gpio_t AuxIn4;
#endif
#ifdef AUXINPUT5_PIN
  static gpio_t AuxIn5;
#endif
#ifdef AUXINPUT6_PIN
  static gpio_t AuxIn6;
#endif
#ifdef AUXINPUT7_PIN
  static gpio_t AuxIn7;
#endif

#ifdef AUXOUTPUT0_PIN
  static gpio_t AuxOut0;
#endif
#ifdef AUXOUTPUT1_PIN
  static gpio_t AuxOut1;
#endif
#ifdef AUXOUTPUT2_PIN
  static gpio_t AuxOut2;
#endif
#ifdef AUXOUTPUT3_PIN
  static gpio_t AuxOut3;
#endif
#ifdef AUXOUTPUT4_PIN
  static gpio_t AuxOut4;
#endif
#ifdef AUXOUTPUT5_PIN
  static gpio_t AuxOut5;
#endif
#ifdef AUXOUTPUT6_PIN
  static gpio_t AuxOut6;
#endif
#ifdef AUXOUTPUT7_PIN
  static gpio_t AuxOut7;
#endif

static periph_signal_t *periph_pins = NULL;

input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,          .port = &Reset,          .pin = RESET_PIN,           .group = PinGroup_Control },
#else
    { .id = Input_Reset,          .port = &Reset,          .pin = RESET_PIN,           .group = PinGroup_Control },
#endif
    { .id = Input_FeedHold,       .port = &FeedHold,       .pin = FEED_HOLD_PIN,       .group = PinGroup_Control },
    { .id = Input_CycleStart,     .port = &CycleStart,     .pin = CYCLE_START_PIN,     .group = PinGroup_Control },
// Limit input pins must be consecutive
    { .id = Input_LimitX,         .port = &LimitX,         .pin = X_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_2,       .port = &LimitX2,        .pin = X2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,     .port = &LimitXMax,      .pin = X_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
    { .id = Input_LimitY,         .port = &LimitY,         .pin = Y_LIMIT_PIN,         .group = PinGroup_Limit },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_2,       .port = &LimitY2,        .pin = Y2_LIMIT_PIN,        .group = PinGroup_Limit },
#endif
#ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,     .port = &LimitYMax,      .pin = Y_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax },
#endif
    { .id = Input_LimitZ,         .port = &LimitZ,         .pin = Z_LIMIT_PIN,         .group = PinGroup_Limit }
#ifdef Z2_LIMIT_PIN
  , { .id = Input_LimitZ_2,       .port = &LimitZ2,        .pin = Z2_LIMIT_PIN,        .group = PinGroup_Limit }
#endif
#ifdef Z_LIMIT_PIN_MAX
  , { .id = Input_LimitZ_Max,     .port = &LimitZMax,      .pin = Z_LIMIT_PIN_MAX,     .group = PinGroup_LimitMax }
#endif
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,         .port = &LimitA,         .pin = A_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,         .port = &LimitB,         .pin = B_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
#ifdef C_LIMIT_PIN
  , { .id = Input_LimitC,         .port = &LimitC,         .pin = C_LIMIT_PIN,         .group = PinGroup_Limit }
#endif
// End limit pin definitions
#ifndef AUX_DEVICES
#ifdef PROBE_PIN
    { .id = Input_Probe,          .port = &Probe,          .pin = PROBE_PIN,           .group = PinGroup_Probe },
#endif
#if MPG_MODE_PIN
  , { .id = Input_ModeSelect,     .port = &ModeSelect,     .pin = MPG_MODE_PIN,        .group = PinGroup_MPG }
#endif
#if I2C_STROBE_ENABLE && defined(I2C_STROBE_PIN)
  , { .id = Input_KeypadStrobe,   .port = &KeypadStrobe,   .pin = I2C_STROBE_PIN,      .group = PinGroup_Keypad }
#endif
#endif // AUX_DEVICES
#ifdef SPINDLE_INDEX_PIN
  , { .id = Input_SpindleIndex,   .port = &SpindleIndex,   .pin = SPINDLE_INDEX_PIN,   .group = PinGroup_SpindleIndex }
#endif
#if QEI_ENABLE
  , { .id = Input_QEI_A,          .port = &QEI_A,          .pin = QEI_A_PIN,           .group = PinGroup_QEI }
  , { .id = Input_QEI_B,          .port = &QEI_B,          .pin = QEI_B_PIN,           .group = PinGroup_QEI }
  #if QEI_SELECT_ENABLED
  , { .id = Input_QEI_Select,     .port = &QEI_Select,     .pin = QEI_SELECT_PIN,      .group = PinGroup_QEI_Select }
  #endif
  #if QEI_INDEX_ENABLED
  , { .id = Input_QEI_Index,      .port = &QEI_Index,      .pin = QEI_INDEX_PIN,       .group = PinGroup_QEI }
  #endif
#endif
// Aux input pins must be consecutive
#ifdef AUXINPUT0_PIN
  , { .id = Input_Aux0,           .port = &AuxIn0,         .pin = AUXINPUT0_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT1_PIN
  , { .id = Input_Aux1,           .port = &AuxIn1,         .pin = AUXINPUT1_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT2_PIN
  , { .id = Input_Aux2,           .port = &AuxIn2,         .pin = AUXINPUT2_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT3_PIN
  , { .id = Input_Aux3,           .port = &AuxIn3,         .pin = AUXINPUT3_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT4_PIN
  , { .id = Input_Aux4,           .port = &AuxIn4,         .pin = AUXINPUT4_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT5_PIN
  , { .id = Input_Aux5,           .port = &AuxIn5,         .pin = AUXINPUT5_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT6_PIN
  , { .id = Input_Aux6,           .port = &AuxIn6,         .pin = AUXINPUT6_PIN,       .group = PinGroup_AuxInput }
#endif
#ifdef AUXINPUT7_PIN
  , { .id = Input_Aux7,           .port = &AuxIn7,         .pin = AUXINPUT7_PIN,       .group = PinGroup_AuxInput }
#endif
};

static output_signal_t outputpin[] = {
    { .id = Output_StepX,           .port = &stepX,         .pin = X_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepY,           .port = &stepY,         .pin = Y_STEP_PIN,              .group = PinGroup_StepperStep },
    { .id = Output_StepZ,           .port = &stepZ,         .pin = Z_STEP_PIN,              .group = PinGroup_StepperStep },
#ifdef A_AXIS
    { .id = Output_StepA,           .port = &stepA,         .pin = A_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef B_AXIS
    { .id = Output_StepB,           .port = &stepB,         .pin = B_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef C_AXIS
    { .id = Output_StepC,           .port = &stepC,         .pin = C_STEP_PIN,              .group = PinGroup_StepperStep },
#endif
#ifdef X2_STEP_PIN
    { .id = Output_StepX_2,         .port = &stepX2,        .pin = X2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Y2_STEP_PIN
    { .id = Output_StepY_2,         .port = &stepY2,        .pin = Y2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
#ifdef Z2_STEP_PIN
    { .id = Output_StepZ_2,         .port = &stepZ2,        .pin = Z2_STEP_PIN,             .group = PinGroup_StepperStep },
#endif
    { .id = Output_DirX,            .port = &dirX,          .pin = X_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirY,            .port = &dirY,          .pin = Y_DIRECTION_PIN,         .group = PinGroup_StepperDir },
    { .id = Output_DirZ,            .port = &dirZ,          .pin = Z_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#ifdef A_AXIS
    { .id = Output_DirA,            .port = &dirA,          .pin = A_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef B_AXIS
    { .id = Output_DirB,            .port = &dirB,          .pin = B_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef C_AXIS
    { .id = Output_DirC,            .port = &dirC,          .pin = C_DIRECTION_PIN,         .group = PinGroup_StepperDir },
#endif
#ifdef X2_DIRECTION_PIN
    { .id = Output_DirX_2,          .port = &dirX2,         .pin = X2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Y2_DIRECTION_PIN
    { .id = Output_DirY_2,          .port = &dirY2,         .pin = Y2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#ifdef Z2_DIRECTION_PIN
    { .id = Output_DirZ_2,          .port = &dirZ2,         .pin = Z2_DIRECTION_PIN,        .group = PinGroup_StepperDir },
#endif
#if !TRINAMIC_ENABLE
#ifdef STEPPERS_ENABLE_PIN
    { .id = Output_StepperEnable,   .port = &steppersEnable, .pin = STEPPERS_ENABLE_PIN,    .group = PinGroup_StepperEnable },
#endif
#ifdef X_ENABLE_PIN
    { .id = Output_StepperEnableX,  .port = &enableX,       .pin = X_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef Y_ENABLE_PIN
    { .id = Output_StepperEnableY,  .port = &enableY,       .pin = Y_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef Z_ENABLE_PIN
    { .id = Output_StepperEnableZ,  .port = &enableZ,       .pin = Z_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef A_ENABLE_PIN
    { .id = Output_StepperEnableA,  .port = &enableA,       .pin = A_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef B_ENABLE_PIN
    { .id = Output_StepperEnableB,  .port = &enableB,       .pin = B_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef C_ENABLE_PIN
    { .id = Output_StepperEnableC,  .port = &enableC,       .pin = C_ENABLE_PIN,            .group = PinGroup_StepperEnable },
#endif
#ifdef X2_ENABLE_PIN
    { .id = Output_StepperEnableX,  .port = &enableX2,      .pin = X2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Y2_ENABLE_PIN
    { .id = Output_StepperEnableY,  .port = &enableY2,      .pin = Y2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#ifdef Z2_ENABLE_PIN
    { .id = Output_StepperEnableZ,  .port = &enableZ2,      .pin = Z2_ENABLE_PIN,           .group = PinGroup_StepperEnable },
#endif
#endif
#if DRIVER_SPINDLE_ENABLE
    { .id = Output_SpindleOn,       .port = &spindleEnable, .pin = SPINDLE_ENABLE_PIN,      .group = PinGroup_SpindleControl },
#ifdef SPINDLE_DIRECTION_PIN
    { .id = Output_SpindleDir,      .port = &spindleDir,    .pin = SPINDLE_DIRECTION_PIN,   .group = PinGroup_SpindleControl },
#endif
#endif // DRIVER_SPINDLE_ENABLE
    { .id = Output_CoolantFlood,    .port = &Flood,         .pin = COOLANT_FLOOD_PIN,       .group = PinGroup_Coolant },
#ifdef COOLANT_MIST_PIN
    { .id = Output_CoolantMist,     .port = &Mist,          .pin = COOLANT_MIST_PIN,        .group = PinGroup_Coolant },
#endif
#ifdef AUXOUTPUT0_PIN
    { .id = Output_Aux0,            .port = &AuxOut0,       .pin = AUXOUTPUT0_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT1_PIN
    { .id = Output_Aux1,            .port = &AuxOut1,       .pin = AUXOUTPUT1_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT2_PIN
    { .id = Output_Aux2,            .port = &AuxOut2,       .pin = AUXOUTPUT2_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT3_PIN
    { .id = Output_Aux3,            .port = &AuxOut3,       .pin = AUXOUTPUT3_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT4_PIN
    { .id = Output_Aux4,            .port = &AuxOut4,       .pin = AUXOUTPUT4_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT5_PIN
    { .id = Output_Aux5,            .port = &AuxOut5,       .pin = AUXOUTPUT5_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT6_PIN
    { .id = Output_Aux6,            .port = &AuxOut6,       .pin = AUXOUTPUT6_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT7_PIN
    { .id = Output_Aux7,            .port = &AuxOut7,       .pin = AUXOUTPUT7_PIN,          .group = PinGroup_AuxOutput },
#endif
#ifdef AUXOUTPUT0_PWM_PIN
    { .id = Output_Analog_Aux0,     .port = NULL,           .pin = AUXOUTPUT0_PWM_PIN,      .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT0_ANALOG_PIN
    { .id = Output_Analog_Aux0,     .port = NULL,           .pin = AUXOUTPUT0_ANALOG_PIN,   .group = PinGroup_AuxOutputAnalog },
#endif
#ifdef AUXOUTPUT1_PWM_PIN
    { .id = Output_Analog_Aux1,     .port = NULL,           .pin = AUXOUTPUT1_PWM_PIN,      .group = PinGroup_AuxOutputAnalog, .mode = { PINMODE_PWM } },
#endif
#ifdef AUXOUTPUT1_ANALOG_PIN
    { .id = Output_Analog_Aux1,     .port = NULL,           .pin = AUXOUTPUT1_ANALOG_PIN,   .group = PinGroup_AuxOutputAnalog }
#endif
};

#if AUX_CONTROLS_ENABLED

#ifdef PROBE_PIN
static uint8_t probe_port;
static input_signal_t *probe_pin;
#endif
#ifdef SAFETY_DOOR_PIN
static input_signal_t *door_pin;
#endif
#ifdef MOTOR_FAULT_PIN
static input_signal_t *motor_fault_pin;
#endif
#ifdef MOTOR_WARNING_PIN
static input_signal_t *motor_warning_pin;
#endif
#ifdef QEI_SELECT_PIN
static input_signal_t *qei_select_pin;
#endif
#ifdef MPG_MODE_PIN
static uint8_t mpg_port;
static input_signal_t *mpg_pin;
#endif

static void aux_irq_handler (uint8_t port, bool state);

#endif // AUX_CONTROLS_ENABLED

static pin_debounce_t debounce = {0};
static pin_group_pins_t limit_inputs = {0};

static bool IOInitDone = false, rtc_started = false;
static uint16_t pulse_length, pulse_delay;
static axes_signals_t next_step_outbits;
static delay_t grbl_delay = { .ms = 0, .callback = NULL };
static probe_state_t probe = {
    .connected = On
};
#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

#if I2C_STROBE_ENABLE

static driver_irq_handler_t i2c_strobe = { .type = IRQ_I2C_Strobe };

static bool irq_claim (irq_type_t irq, uint_fast8_t id, irq_callback_ptr handler)
{
    bool ok;

    if((ok = irq == IRQ_I2C_Strobe && i2c_strobe.callback == NULL))
        i2c_strobe.callback = handler;

    return ok;
}

#endif

// Interrupt handler prototypes
// Interrupt handlers needs to be registered, possibly by modifying a system specific startup file.
// It is possible to relocate the interrupt dispatch table from flash to RAM and programatically attach handlers.
// See the driver for SAMD21 for an example, relocation is done in the driver_init() function.
// Also, if a MCU specific driver library is used this might have functions to programatically attach handlers.

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void stepper_pulse_isr_delayed (void);
static void gpio_isr (void);
static void systick_isr (void);

static void (*systick_isr_org)(void) = NULL;

// Millisecond resolution delay function
// Will return immediately if a callback function is provided
static void driver_delay_ms (uint32_t ms, delay_callback_ptr callback)
{
    if(ms) {
        grbl_delay.ms = ms;
        if(!(grbl_delay.callback = callback)) {
            while(grbl_delay.ms)
                grbl.on_execute_delay(state_get());
        }
    } else {
        if(grbl_delay.ms) {
            grbl_delay.callback = NULL;
            grbl_delay.ms = 1;
        }
        if(callback)
            callback();
    }
}

// Set stepper pulse output pins.
// step_outbits.value (or step_outbits.mask) are: bit0 -> X, bit1 -> Y...
// Individual step bits can be accessed by step_outbits.x, step_outbits.y, ...
#ifdef SQUARING_ENABLED

inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;

    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    DIGITAL_OUT(stepX, step_outbits_1.x);
#ifdef X2_STEP_PIN
    DIGITAL_OUT(stepX2, step_outbits_2.x);
#endif

    DIGITAL_OUT(stepY, step_outbits_1.y);
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(stepY2, step_outbits_2.y);
#endif

    DIGITAL_OUT(stepZ, step_outbits_1.z);
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(stepZ2, step_outbits_2.z);
#endif

#ifdef A_AXIS
    DIGITAL_OUT(stepA, step_outbits_1.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(stepB, step_outbits_1.b);
#endif
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#else

inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;

    DIGITAL_OUT(stepX, step_outbits.x);
#ifdef X2_STEP_PIN
    DIGITAL_OUT(stepX2, step_outbits.x);
#endif

    DIGITAL_OUT(stepY, step_outbits.y);
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(stepY2, step_outbits.y);
#endif

    DIGITAL_OUT(stepZ, step_outbits.z);
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(stepZ2, step_outbits.z);
#endif

#ifdef A_AXIS
    DIGITAL_OUT(stepA, step_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(stepB, step_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(stepC, step_outbits.c);
#endif
}

#endif // SQUARING_ENABLED

#ifdef GANGING_ENABLED

static axes_signals_t getGangedAxes (bool auto_squared)
{
    axes_signals_t ganged = {0};

    if(auto_squared) {
        #if X_AUTO_SQUARE
            ganged.x = On;
        #endif
        #if Y_AUTO_SQUARE
            ganged.y = On;
        #endif
        #if Z_AUTO_SQUARE
            ganged.z = On;
        #endif
    } else {
        #if X_GANGED
            ganged.x = On;
        #endif

        #if Y_GANGED
            ganged.y = On;
        #endif

        #if Z_GANGED
            ganged.z = On;
        #endif
    }

    return ganged;
}

#endif

// Set stepper direction ouput pins.
// dir_outbits.value (or dir_outbits.mask) are: bit0 -> X, bit1 -> Y...
// Individual direction bits can be accessed by dir_outbits.x, dir_outbits.y, ...
inline static __attribute__((always_inline)) void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;

    DIGITAL_OUT(dirX, dir_outbits.x);
    DIGITAL_OUT(dirY, dir_outbits.y);
    DIGITAL_OUT(dirZ, dir_outbits.z);

#ifdef GANGING_ENABLED
    dir_outbits.mask ^= settings.steppers.ganged_dir_invert.mask;
  #ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(dirX2, dir_outbits.x);
  #endif
  #ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(dirY2, dir_outbits.y);
  #endif
  #ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(dirZ2, dir_outbits.z);
#endif
#endif

#ifdef A_AXIS
    DIGITAL_OUT(dirA, dir_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(dirB, dir_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(dirC, dir_outbits.c);
#endif
}

// Enable steppers.
// enable.value (or enable.mask) are: bit0 -> X, bit1 -> Y...
// Individual enable bits can be accessed by enable.x, enable.y, ...
// NOTE: if a common signal is used to enable all drivers enable.x should be used to set the signal.
static void stepperEnable (axes_signals_t enable, bool hold)
{
    enable.value ^= settings.steppers.enable_invert.mask;

#ifdef STEPPERS_ENABLE_PIN
    DIGITAL_OUT(steppersEnable, enable.x)
#endif

#ifdef X_ENABLE_PIN
    DIGITAL_OUT(enableX, enable.x)
#endif
#ifdef X2_ENABLE_PIN
    DIGITAL_OUT(enableX2, enable.x)
#endif

#ifdef Y_ENABLE_PIN
    DIGITAL_OUT(enableY, enable.y)
#endif
#ifdef Y2_ENABLE_PIN
    DIGITAL_OUT(enableY2, enable.y)
#endif

#ifdef Z_ENABLE_PIN
    DIGITAL_OUT(enableZ, enable.z)
#endif
#ifdef Z2_ENABLE_PIN
    DIGITAL_OUT(enableZ2, enable.z)
#endif

#ifdef A_ENABLE_PIN
    DIGITAL_OUT(enableA, enable.a)
#endif
#ifdef B_ENABLE_PIN
    DIGITAL_OUT(enableB, enable.b)
#endif
#ifdef C_ENABLE_PIN
    DIGITAL_OUT(enableC, enable.c)
#endif
}

// Starts stepper driver timer and forces a stepper driver interrupt callback.
static void stepperWakeUp (void)
{
    // Enable stepper drivers.
    hal.stepper.enable((axes_signals_t){AXES_BITMASK}, false);

    PIT_LDVAL0 = hal.f_step_timer / 500; // ~2ms delay to allow drivers time to wake up.
    PIT_TFLG0 |= PIT_TFLG_TIF;
    PIT_TCTRL0 |= (PIT_TCTRL_TIE|PIT_TCTRL_TEN);
}

// Disables stepper driver interrupts and reset outputs.
static void stepperGoIdle (bool clear_signals)
{
    PIT_TCTRL0 &= ~(PIT_TCTRL_TIE|PIT_TCTRL_TEN);

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout.
// Called at the start of each segment.
// NOTE: If a 32-bit timer is used it is advisable to limit max step time to about 2 seconds
//       in order to avoid excessive delays on completion of motions
// NOTE: If a 16 bit timer is used it may be neccesary to adjust the timer clock frequency (prescaler)
//       to cover the needed range. Refer to actual drivers for code examples.
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN;
    PIT_LDVAL0 = cycles_per_tick < (1UL << 20) ? cycles_per_tick : 0x000FFFFFUL;
    PIT_TFLG0 |= PIT_TFLG_TIF;
    PIT_TCTRL0 |= PIT_TCTRL_TEN;
}

// Start a stepper pulse, no delay version.
// stepper_t struct is defined in grbl/stepper.h
static void stepperPulseStart (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block && stepper->exec_segment->spindle_sync) {
        spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStartSynchronized;
        hal.stepper.pulse_start(stepper);
        return;
    }
#endif

    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER_CTRL |= TMR_CTRL_CM(0b001);
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
//       In the delayed step pulse interrupt handler the pulses are output and
//       normal (no delay) operation is resumed.
// stepper_t struct is defined in grbl/stepper.h
static void stepperPulseStartDelayed (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block && stepper->exec_segment->spindle_sync) {
        spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStartSynchronized;
        hal.stepper.pulse_start(stepper);
        return;
    }
#endif

    if(stepper->dir_change) {

        set_dir_outputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {

            next_step_outbits = stepper->step_outbits; // Store out_bits

            attachInterruptVector(PULSE_TIMER_IRQ, stepper_pulse_isr_delayed);

            PULSE_TIMER_COMP1 = pulse_delay;
            PULSE_TIMER_CTRL |= TMR_CTRL_CM(0b001);
        }

        return;
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER_CTRL |= TMR_CTRL_CM(0b001);
    }
}

#if SPINDLE_SYNC_ENABLE

// Spindle sync version: sets stepper direction and pulse pins and starts a step pulse.
// Switches back to "normal" version if spindle synchronized motion is finished.
// TODO: add delayed pulse handling...
static void stepperPulseStartSynchronized (stepper_t *stepper)
{
    static bool sync = false;
    static float block_start;

    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper.pulse_start = spindle_tracker.stepper_pulse_start_normal;
            hal.stepper.pulse_start(stepper);
            return;
        }
        sync = true;
        set_dir_outputs(stepper->dir_outbits);
        spindle_tracker.programmed_rate = stepper->exec_block->programmed_rate;
        spindle_tracker.steps_per_mm = stepper->exec_block->steps_per_mm;
        spindle_tracker.segment_id = 0;
        spindle_tracker.prev_pos = 0.0f;
        block_start = stepper->exec_block->spindle->get_data(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;
        pidf_reset(&spindle_tracker.pid);
#ifdef PID_LOG
        sys.pid_log.idx = 0;
        sys.pid_log.setpoint = 100.0f;
#endif
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER_CTRL |= TMR_CTRL_CM(0b001);
    }

    if(spindle_tracker.segment_id != stepper->exec_segment->id) {

        spindle_tracker.segment_id = stepper->exec_segment->id;

        if(!stepper->new_block) {  // adjust this segments total time for any positional error since last segment

            float actual_pos;

            if(stepper->exec_segment->cruising) {

                float dt = (float)hal.f_step_timer / (float)(stepper->exec_segment->cycles_per_tick * stepper->exec_segment->n_step);
                actual_pos = stepper->exec_block->spindle->get_data(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;

                if(sync) {
                    spindle_tracker.pid.sample_rate_prev = dt;
//                    block_start += (actual_pos - spindle_tracker.block_start) - spindle_tracker.prev_pos;
//                    block_start += spindle_tracker.prev_pos;
                    sync = false;
                }

                actual_pos -= block_start;
                int32_t step_delta = (int32_t)(pidf(&spindle_tracker.pid, spindle_tracker.prev_pos, actual_pos, dt) * spindle_tracker.steps_per_mm);


                int32_t ticks = (((int32_t)stepper->step_count + step_delta) * (int32_t)stepper->exec_segment->cycles_per_tick) / (int32_t)stepper->step_count;

                stepper->exec_segment->cycles_per_tick = (uint32_t)max(ticks, spindle_tracker.min_cycles_per_tick);

                stepperCyclesPerTick(stepper->exec_segment->cycles_per_tick);
           } else
               actual_pos = spindle_tracker.prev_pos;

#ifdef PID_LOG
            if(sys.pid_log.idx < PID_LOG) {

                sys.pid_log.target[sys.pid_log.idx] = spindle_tracker.prev_pos;
                sys.pid_log.actual[sys.pid_log.idx] = actual_pos; // - spindle_tracker.prev_pos;

            //    spindle_tracker.log[sys.pid_log.idx] = STEPPER_TIMER->BGLOAD << stepper->amass_level;
            //    spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick  stepper->amass_level;
            //    spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick * stepper->step_count;
            //    STEPPER_TIMER->BGLOAD = STEPPER_TIMER->LOAD;

             //   spindle_tracker.pos[sys.pid_log.idx] = spindle_tracker.prev_pos;

                sys.pid_log.idx++;
            }
#endif
        }

        spindle_tracker.prev_pos = stepper->exec_segment->target_position;
    }
}

#endif

#if STEP_INJECT_ENABLE

static void output_pulse_isr (void);
static void output_pulse_isr_delayed (void);

static volatile axes_signals_t pulse_output = {0};

static inline __attribute__((always_inline)) void stepperInjectStep (axes_signals_t step_outbits)
{
    if(pulse_output.x) {
        DIGITAL_OUT(stepX, step_outbits.x);
#ifdef X2_STEP_PIN
        DIGITAL_OUT(stepX2, step_outbits.x);
#endif
    }

    if(pulse_output.y) {
        DIGITAL_OUT(stepY, step_outbits.y);
#ifdef Y2_STEP_PIN
        DIGITAL_OUT(stepY2, step_outbits.y);
#endif
    }

    if(pulse_output.z) {
        DIGITAL_OUT(stepZ, step_outbits.z);
#ifdef Z2_STEP_PIN
        DIGITAL_OUT(stepZ2, step_outbits.z);
#endif
    }

#ifdef A_AXIS
    if(pulse_output.a)
        DIGITAL_OUT(stepA, step_outbits.a);

#endif
#ifdef B_AXIS
    if(pulse_output.b)
        DIGITAL_OUT(stepB, step_outbits.b);
#endif
}

void stepperOutputStep (axes_signals_t step_outbits, axes_signals_t dir_outbits)
{
    if(pulse_output.value)
        return;

    if(step_outbits.value) {

        pulse_output = step_outbits;
        dir_outbits.value ^= settings.steppers.dir_invert.mask;

        if(pulse_output.x)
            DIGITAL_OUT(dirX, dir_outbits.z);

        if(pulse_output.y)
            DIGITAL_OUT(dirY, dir_outbits.y);

        if(pulse_output.z)
            DIGITAL_OUT(dirZ, dir_outbits.z);

#ifdef A_AXIS
        if(pulse_output.a)
            DIGITAL_OUT(dirA, dir_outbits.a);
#endif
#ifdef B_AXIS
        if(pulse_output.b)
            DIGITAL_OUT(dirB, dir_outbits.b);
#endif

        if(pulse_delay) {
            attachInterruptVector(PULSE2_TIMER_IRQ, output_pulse_isr_delayed);
            PULSE2_TIMER_COMP1 = pulse_delay;
            PULSE2_TIMER_CTRL |= TMR_CTRL_CM(0b001);
        } else {
            step_outbits.value ^= settings.steppers.step_invert.mask;
            stepperInjectStep(step_outbits);
            PULSE2_TIMER_CTRL |= TMR_CTRL_CM(0b001);
        }
    }
}

#endif

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
// Dual limit switch inputs per axis version. Only one needs to be dual input!
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
    signals.min2.mask = settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
    signals.max.mask = settings.limits.invert.mask;
#endif

    signals.min.x = DIGITAL_IN(LimitX);
#ifdef X2_LIMIT_PIN
    signals.min2.x = DIGITAL_IN(LimitX2);
#endif
#ifdef X_LIMIT_PIN_MAX
    signals.max.x = DIGITAL_IN(LimitXMax);
#endif

    signals.min.y = DIGITAL_IN(LimitY);
#ifdef Y2_LIMIT_PIN
    signals.min2.y = DIGITAL_IN(LimitY2);
#endif
#ifdef Y_LIMIT_PIN_MAX
    signals.max.y = DIGITAL_IN(LimitYMax);
#endif

    signals.min.z = DIGITAL_IN(LimitZ);
#ifdef Z2_LIMIT_PIN
    signals.min2.z = DIGITAL_IN(LimitZ2);
#endif
#ifdef Z_LIMIT_PIN_MAX
    signals.max.z = DIGITAL_IN(LimitZMax);
#endif

#ifdef A_LIMIT_PIN
    signals.min.a = DIGITAL_IN(LimitA);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = DIGITAL_IN(LimitB);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = DIGITAL_IN(LimitC);
#endif

    if(settings.limits.invert.mask) {
        signals.min.mask ^= settings.limits.invert.mask;
#ifdef DUAL_LIMIT_SWITCHES
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
#ifdef MAX_LIMIT_SWITCHES
        signals.max.value ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

// Enable/disable limit pins interrupt.
// NOTE: the homing parameter is indended for configuring advanced
//        stepper drivers for sensorless homing.
static void limitsEnable (bool on, axes_signals_t homing_cycle)
{
    bool disable = !on;
    uint32_t i = limit_inputs.n_pins;
    axes_signals_t pin;
    limit_signals_t homing_source = xbar_get_homing_source_from_cycle(homing_cycle);

    do {
        i--;
        limit_inputs.pins.inputs[i].gpio.reg->ISR = limit_inputs.pins.inputs[i].gpio.bit;       // Clear interrupt.
        if(on && homing_cycle.mask) {
            pin = xbar_fn_to_axismask(limit_inputs.pins.inputs[i].id);
            disable = limit_inputs.pins.inputs[i].group == PinGroup_Limit ? (pin.mask & homing_source.min.mask) : (pin.mask & homing_source.max.mask);
        }
        if(disable)
            limit_inputs.pins.inputs[i].gpio.reg->IMR &= ~limit_inputs.pins.inputs[i].gpio.bit; // Disable interrupt.
        else
            limit_inputs.pins.inputs[i].gpio.reg->IMR |= limit_inputs.pins.inputs[i].gpio.bit;  // Enable interrupt.
    } while(i);
}

// Returns system state as a control_signals_t bitmap variable.
// signals.value (or signals.mask) are: bit0 -> reset, bit1 -> feed_hold, ...
// Individual enable bits can be accessed by signals.reset, signals.feed_hold, ...
// Each bit indicates a control signal, where triggered is 1 and not triggered is 0.
// axes_signals_t is defined in grbl/system.h.
inline static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

#if ESTOP_ENABLE
    signals.e_stop = DIGITAL_IN(Reset);
#else
    signals.reset = DIGITAL_IN(Reset);
#endif
    signals.feed_hold = DIGITAL_IN(FeedHold);
    signals.cycle_start = DIGITAL_IN(CycleStart);

#if AUX_CONTROLS_ENABLED

  #ifdef SAFETY_DOOR_PIN
    if(debounce.safety_door)
        signals.safety_door_ajar = !settings.control_invert.safety_door_ajar;
    else
        signals.safety_door_ajar = DIGITAL_IN(door_pin->gpio);
  #endif
  #ifdef MOTOR_FAULT_PIN
    signals.motor_fault = DIGITAL_IN(motor_fault_pin->gpio);
  #endif
  #ifdef MOTOR_WARNING_PIN
    signals.motor_warning = DIGITAL_IN(motor_warning_pin->gpio);
  #endif

  if(settings.control_invert.mask)
      signals.value ^= settings.control_invert.mask;

#else

  #ifdef SAFETY_DOOR_PIN
    if(!door_pin->debounce)
        signals.safety_door_ajar = DIGITAL_IN(door_pin->gpio);
  #endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

#endif

    return signals;
}

#if PROBE_ENABLE

// Toggle probe connected status. Used when no input pin is available.
static void probeConnectedToggle (void)
{
    probe.connected = !probe.connected;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

    if(hal.driver_cap.probe_latch) {
        probe.is_probing = Off;
        probe.triggered = hal.probe.get_state().triggered;
        pin_irq_mode_t irq_mode = probing && !probe.triggered ? (probe.inverted ? IRQ_Mode_Falling : IRQ_Mode_Rising) : IRQ_Mode_None;
        probe.irq_enabled = hal.port.register_interrupt_handler(probe_port, irq_mode, aux_irq_handler) && irq_mode != IRQ_Mode_None;
    }

    if(!probe.irq_enabled)
        probe.triggered = Off;

    probe.is_probing = probing;
}

// Returns the probe connected and triggered pin states.
static probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
 //   state.triggered = !!(Probe.reg->DR & Probe.bit) ^ probe.inverted;
    state.triggered = probe.is_probing && probe.irq_enabled ? probe.triggered : DIGITAL_IN(probe_pin->gpio) ^ probe.inverted;

    return state;
}

#endif // PROBE_ENABLE

#if MPG_ENABLE == 1

static void mpg_select (void *data)
{
    stream_mpg_enable(DIGITAL_IN(mpg_pin->gpio) == 0);

    hal.port.register_interrupt_handler(mpg_port, mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling, aux_irq_handler);
//    pinEnableIRQ(mpg_pin, (mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

static void mpg_enable (void *data)
{
    if(sys.mpg_mode == DIGITAL_IN(mpg_pin->gpio))
        stream_mpg_enable(true);

    hal.port.register_interrupt_handler(mpg_port, mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling, aux_irq_handler);
//    pinEnableIRQ(mpg_pin, (mpg_pin->mode.irq_mode = sys.mpg_mode ? IRQ_Mode_Rising : IRQ_Mode_Falling));
}

#endif

#if QEI_SELECT_ENABLE

static void qei_select_handler (void)
{
//    if(DIGITAL_IN(QEI_SELECT_PORT, QEI_SELECT_PIN))
//        return;

    if(!qei.dbl_click_timeout)
        qei.dbl_click_timeout = qei.encoder.settings->dbl_click_window;
    else if(qei.dbl_click_timeout < qei.encoder.settings->dbl_click_window - 40) {
        qei.dbl_click_timeout = 0;
        qei.encoder.event.dbl_click = On;
        hal.encoder.on_event(&qei.encoder, qei.count);
    }
}

#endif

#if AUX_CONTROLS_ENABLED

static void aux_irq_handler (uint8_t port, bool state)
{
    aux_ctrl_t *pin;
    control_signals_t signals = {0};
    
    if((pin = aux_ctrl_get_pin(port))) {
        switch(pin->function) {
#ifdef PROBE_PIN
            case Input_Probe:
                if(probe.is_probing) {
                    probe.triggered = On;
                    return;
                } else
                    signals.probe_triggered = On;
                break;
#endif
#ifdef QEI_SELECT_PIN
            case Input_QEI_Select:
                qei_select_handler();
                break;
#endif
#ifdef I2C_STROBE_PIN
            case Input_I2CStrobe:
                if(i2c_strobe.callback)
                    i2c_strobe.callback(0, DIGITAL_IN(((input_signal_t *)pin->input)->gpio) == 0);
                break;
#endif
#ifdef MPG_MODE_PIN
            case Input_MPGSelect:
                protocol_enqueue_foreground_task(mpg_select, NULL);
                break;
#endif
            default:
                break;
        }
        signals.mask |= pin->cap.mask;
        if(pin->irq_mode == IRQ_Mode_Change && pin->function != Input_Probe)
            signals.deasserted = hal.port.wait_on_input(Port_Digital, pin->aux_port, WaitMode_Immediate, 0.0f) == 0;
    }

    if(signals.mask) {
        if(!signals.deasserted)
            signals.mask |= systemGetState().mask;
        hal.control.interrupt_callback(signals);
    }
}

static bool aux_claim_explicit (aux_ctrl_t *aux_ctrl)
{
    if(ioport_claim(Port_Digital, Port_Input, &aux_ctrl->aux_port, NULL)) {
        ioport_assign_function(aux_ctrl, &((input_signal_t *)aux_ctrl->input)->id);
#ifdef PROBE_PIN
        if(aux_ctrl->function == Input_Probe) {
            probe_port = aux_ctrl->aux_port;
            probe_pin = (input_signal_t *)aux_ctrl->input;
            hal.probe.get_state = probeGetState;
            hal.probe.configure = probeConfigure;
            hal.probe.connected_toggle = probeConnectedToggle;
            hal.driver_cap.probe_pull_up = On;
            hal.signals_cap.probe_triggered = hal.driver_cap.probe_latch = aux_ctrl->irq_mode != IRQ_Mode_None;
        }
#endif
#ifdef SAFETY_DOOR_PIN
        if(aux_ctrl->function == Input_SafetyDoor) {
            door_pin = (input_signal_t *)aux_ctrl->input;
            door_pin->mode.debounce = hal.driver_cap.software_debounce;
        }
#endif
#ifdef MOTOR_FAULT_PIN
        if(aux_ctrl->function == Input_MotorFault)
            motor_fault_pin = (input_signal_t *)aux_ctrl->input;
#endif
#ifdef MOTOR_WARNING_PIN
        if(aux_ctrl->function == Input_MotorWarning)
            motor_warning_pin = (input_signal_t *)aux_ctrl->input;
#endif
#ifdef QEI_SELECT_PIN
        if(aux_ctrl->function == Input_QEI_Select) {
            qei_select_pin = (input_signal_t *)aux_ctrl->input;
            qei_select_pin->mode.debounce = hal.driver_cap.software_debounce;
        }
#endif
#ifdef MPG_MODE_PIN
        if(aux_ctrl->function == Input_MPGSelect) {
            mpg_port = aux_ctrl->aux_port;
            mpg_pin = (input_signal_t *)aux_ctrl->input;
        }
#endif
    } else
        aux_ctrl->aux_port = 0xFF;

    return aux_ctrl->aux_port != 0xFF;
}

#endif

#if DRIVER_SPINDLE_ENABLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    DIGITAL_OUT(spindleEnable, settings.spindle.invert.on);
}

inline static void spindle_on (spindle_ptrs_t *spindle)
{
    DIGITAL_OUT(spindleEnable, !settings.spindle.invert.on);
#if SPINDLE_ENCODER_ENABLE
    if(spindle && spindle->reset_data)
        spindle->reset_data();
#endif
}

inline static void spindle_dir (bool ccw)
{
#ifdef SPINDLE_DIRECTION_PIN
    DIGITAL_OUT(spindleDir, ccw ^ settings.spindle.invert.ccw);
#else
    UNUSED(ccw);
#endif
}

// Start or stop spindle.
static void spindleSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on(spindle);
    }
}

// Variable spindle control functions

#ifdef SPINDLE_PWM_PIN

// Set spindle speed.
static void spindleSetSpeed (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    if(pwm_value == spindle->context.pwm->off_value) {
        if(spindle->context.pwm->settings->flags.enable_rpm_controlled) {
            if(spindle->context.pwm->cloned)
                spindle_dir(false);
            else
                spindle_off();
        }
        pwmEnabled = false;
        if(spindle->context.pwm->always_on) {
            SPINDLE_PWM_TIMER_COMP2 = spindle->context.pwm->off_value;
            SPINDLE_PWM_TIMER_CMPLD1 = spindle->context.pwm->period - spindle->context.pwm->off_value;
            SPINDLE_PWM_TIMER_CTRL |= TMR_CTRL_CM(0b001);
        } else {
            SPINDLE_PWM_TIMER_CTRL &= ~TMR_CTRL_CM(0b111);
            SPINDLE_PWM_TIMER_SCTRL &= ~TMR_SCTRL_VAL;
            SPINDLE_PWM_TIMER_SCTRL |= TMR_SCTRL_FORCE;
        }
     } else {
        if(!pwmEnabled) {
            if(spindle->context.pwm->cloned)
                spindle_dir(true);
            else
                spindle_on(spindle);
            pwmEnabled = true;
        }
        SPINDLE_PWM_TIMER_COMP2 = pwm_value;
        SPINDLE_PWM_TIMER_CMPLD1 = spindle->context.pwm->period - pwm_value;
        SPINDLE_PWM_TIMER_CTRL |= TMR_CTRL_CM(0b001);
    }
}

// Convert spindle speed to PWM value.
static uint_fast16_t spindleGetPWM (spindle_ptrs_t *spindle, float rpm)
{
    return spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false);
}

// Start or stop spindle.
static void spindleSetStateVariable (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
#ifdef SPINDLE_DIRECTION_PIN
    if(state.on || spindle->context.pwm->cloned)
        spindle_dir(state.ccw);
#endif
    if(!settings.spindle.flags.enable_rpm_controlled) {
        if(state.on)
            spindle_on(spindle);
        else
            spindle_off();
    }

    spindleSetSpeed(spindle, state.on || (state.ccw && spindle->context.pwm->cloned)
                              ? spindle->context.pwm->compute_value(spindle->context.pwm, rpm, false)
                              : spindle->context.pwm->off_value);

#if SPINDLE_ENCODER_ENABLE
    spindle_set_at_speed_range(spindle, &spindle_data, rpm);
#endif
}

bool spindleConfig (spindle_ptrs_t *spindle)
{
    uint_fast16_t prescaler = 2, divider = 0b1001;

    if(spindle == NULL)
        return false;

    if(spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.spindle, F_BUS_ACTUAL / prescaler)) {

        while(spindle_pwm.period > 65534 && divider < 15) {
            prescaler <<= 1;
            divider++;
            spindle_precompute_pwm_values(spindle, &spindle_pwm, &settings.spindle, F_BUS_ACTUAL / prescaler);
        }

        SPINDLE_PWM_TIMER_CTRL = TMR_CTRL_PCS(divider) | TMR_CTRL_OUTMODE(0b100) | TMR_CTRL_LENGTH;
        SPINDLE_PWM_TIMER_COMP1 = spindle_pwm.period;
        SPINDLE_PWM_TIMER_CMPLD1 = spindle_pwm.period;
        if(settings.spindle.invert.pwm)
            SPINDLE_PWM_TIMER_SCTRL |= TMR_SCTRL_OPS;
        else
            SPINDLE_PWM_TIMER_SCTRL &= ~TMR_SCTRL_OPS;

        spindle->set_state = spindleSetStateVariable;
    } else {
        if(pwmEnabled)
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        spindle->set_state = spindleSetState;
    }

    spindle_update_caps(spindle, spindle->cap.variable ? &spindle_pwm : NULL);

    return true;
}

#if PPI_ENABLE

static void spindlePulseOn (uint_fast16_t pulse_length)
{
    static uint_fast16_t plen = 0;

    if(plen != pulse_length) {
        plen = pulse_length;
        PPI_TIMER_COMP1 = (uint16_t)((pulse_length * F_BUS_MHZ) / 128);
    }

    spindle_on(NULL);
    PPI_TIMER_CTRL |= TMR_CTRL_CM(0b001);
}

#endif

#endif // DRIVER_SPINDLE_PWM_ENABLE

// Returns spindle state in a spindle_state_t variable.
// spindle_state_t is defined in grbl/spindle_control.h
static spindle_state_t spindleGetState (spindle_ptrs_t *spindle)
{
    spindle_state_t state = {settings.spindle.invert.mask};

    state.on = (spindleEnable.reg->DR & spindleEnable.bit) != 0;
#ifdef SPINDLE_DIRECTION_PIN
    state.ccw = (spindleDir.reg->DR & spindleDir.bit) != 0;
#endif

    state.value ^= settings.spindle.invert.mask;

#ifdef SPINDLE_PWM_PIN
    if(pwmEnabled)
        state.on |= pwmEnabled;
#endif

#if SPINDLE_ENCODER_ENABLE
    if(spindle && spindle->get_data) {
        float rpm = spindle->get_data(SpindleData_RPM)->rpm;
        state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit);
        state.encoder_error = spindle_encoder.error_count > 0;
    }
#else
    UNUSED(spindle);
#endif

    return state;
}

#endif // DRIVER_SPINDLE_ENABLE

#if SPINDLE_ENCODER_ENABLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;
    spindle_encoder_counter_t encoder;

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = GPT1_CNT - spindle_encoder.timer.last_pulse;

    __enable_irq();

    // If no (4) spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (GPT2_CNT - spindle_encoder.counter.last_count) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.pulse_count = GPT2_CNT;
            spindle_data.index_count = encoder.index_count;
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AtSpeed:
            spindle_validate_at_speed(spindle_data, stopped ? 0.0f : spindle_encoder.rpm_factor / (float)pulse_length);
            spindle_data.state_programmed.encoder_error = spindle_encoder.error_count > 0;
            break;

        case SpindleData_AngularPosition:;
            while(spindleLock);
            int32_t d = encoder.last_count - encoder.last_index;
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)(d) +
                             (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindleLock);

    uint32_t timeout = millis() + 1000; // 1 second

    uint32_t index_count = spindle_data.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_data.index_count && millis() <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }

    GPT2_CR &= ~GPT_CR_EN;  // Reset timer
    GPT1_CR &= ~GPT_CR_EN;  // Reset timer
    GPT1_PR = 24;
    GPT1_CR |= GPT_CR_EN;

    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = GPT1_CNT;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    // Spindle pulse counter
    GPT2_OCR1 = spindle_encoder.tics_per_irq;
    GPT2_CR |= GPT_CR_EN;
}

static void onSpindleProgrammed (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode)
{
    if(on_spindle_programmed)
        on_spindle_programmed(spindle, state, rpm, mode);

    if(spindle->get_data == spindleGetData) {
        spindle_set_at_speed_range(spindle, &spindle_data, rpm);
        spindle_data.state_programmed.on = state.on;
        spindle_data.state_programmed.ccw = state.ccw;
    }
}

#endif // SPINDLE_ENCODER_ENABLE

// Start/stop coolant (and mist if enabled).
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;

    DIGITAL_OUT(Flood, mode.flood);
#ifdef COOLANT_MIST_PIN
    DIGITAL_OUT(Mist, mode.mist);
#endif
}

// Returns coolant state in a coolant_state_t variable.
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = { settings.coolant_invert.mask };

    state.flood = (Flood.reg->DR & Flood.bit) != 0;
#ifdef COOLANT_MIST_PIN
    state.mist = (Mist.reg->DR & Mist.bit) != 0;
#endif
    state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();

    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();

    return prev;
}

static void enable_irq (void)
{
    __enable_irq();
}

static void disable_irq (void)
{
    __disable_irq();
}

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    if(IOInitDone) {

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

#if SPINDLE_ENCODER_ENABLE

        static const spindle_data_ptrs_t encoder_data = {
            .get = spindleGetData,
            .reset = spindleDataReset
        };

        static bool event_claimed = false;

        if((hal.spindle_data.get = settings->spindle.ppr > 0 ? spindleGetData : NULL) &&
             (spindle_encoder.ppr != settings->spindle.ppr || pidf_config_changed(&spindle_tracker.pid, &settings->position.pid))) {

            spindle_ptrs_t *spindle;

            hal.spindle_data.reset = spindleDataReset;
            if((spindle = spindle_get(0)))
                spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);

            pidf_init(&spindle_tracker.pid, &settings->position.pid);

            if(!event_claimed) {
                event_claimed = true;
                on_spindle_programmed = grbl.on_spindle_programmed;
                grbl.on_spindle_programmed = onSpindleProgrammed;
            }

            float timer_resolution = 1.0f / 1000000.0f; // 1 us resolution

            spindle_tracker.min_cycles_per_tick = (int32_t)ceilf(settings->steppers.pulse_microseconds * 2.0f + settings->steppers.pulse_delay_microseconds);
            spindle_encoder.ppr = settings->spindle.ppr;
            spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
            spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
            spindle_encoder.maximum_tt = (uint32_t)(2.0f / timer_resolution) / spindle_encoder.tics_per_irq;
            spindle_encoder.rpm_factor = 60.0f / ((timer_resolution * (float)spindle_encoder.ppr));
            spindleDataReset();
        } else {
            spindle_encoder.ppr = 0;
            hal.spindle_data.reset = NULL;
        }

        spindle_bind_encoder(spindle_encoder.ppr ? &encoder_data : NULL);

#endif // SPINDLE_ENCODER_ENABLE

#if DRIVER_SPINDLE_PWM_ENABLE
        if(changed.spindle) {
            spindleConfig(spindle_get_hal(spindle_id, SpindleHAL_Configured));
            if(spindle_id == spindle_get_default())
                spindle_select(spindle_id);
        }
#endif

        // Stepper pulse timeout setup.
        PULSE_TIMER_CSCTRL &= ~(TMR_CSCTRL_TCF1|TMR_CSCTRL_TCF2);

        pulse_length = (uint16_t)((float)F_BUS_MHZ * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY));

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            float delay = settings->steppers.pulse_delay_microseconds - STEP_PULSE_LATENCY;
            if(delay <= STEP_PULSE_LATENCY)
                delay = STEP_PULSE_LATENCY + 0.2f;
            pulse_delay = (uint16_t)((float)F_BUS_MHZ * delay);
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        PULSE_TIMER_COMP1 = pulse_length;
        PULSE_TIMER_CSCTRL &= ~TMR_CSCTRL_TCF2EN;
        PULSE_TIMER_CTRL &= ~TMR_CTRL_OUTMODE(0b000);
        attachInterruptVector(PULSE_TIMER_IRQ, stepper_pulse_isr);

#if STEP_INJECT_ENABLE
        PULSE2_TIMER_CSCTRL &= ~(TMR_CSCTRL_TCF1|TMR_CSCTRL_TCF2);
        PULSE2_TIMER_COMP1 = pulse_length;
        PULSE2_TIMER_CSCTRL &= ~TMR_CSCTRL_TCF2EN;
        PULSE2_TIMER_CTRL &= ~TMR_CTRL_OUTMODE(0b000);
#endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *signal;

        NVIC_DISABLE_IRQ(IRQ_GPIO6789);

        do {

            signal = &inputpin[--i];
            if(signal->group != PinGroup_AuxInput)
                signal->mode.irq_mode = IRQ_Mode_None;

            switch(signal->id) {
#if ESTOP_ENABLE
                case Input_EStop:
                    signal->mode.pull_mode = settings->control_disable_pullup.e_stop ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = control_fei.e_stop ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#else
                case Input_Reset:
                    signal->mode.pull_mode = settings->control_disable_pullup.reset ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
                case Input_FeedHold:
                    signal->mode.pull_mode = settings->control_disable_pullup.feed_hold ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_CycleStart:
                    signal->mode.pull_mode = settings->control_disable_pullup.cycle_start ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_Probe:
                    signal->mode.pull_mode = settings->probe.disable_probe_pullup ? PullMode_Down : PullMode_Up;
                    break;

                case Input_LimitX:
                case Input_LimitX_2:
                case Input_LimitX_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.x ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                case Input_LimitY_2:
                case Input_LimitY_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.y ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_2:
                case Input_LimitZ_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.z ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                case Input_LimitA_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.a ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
                case Input_LimitB_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef C_LIMIT_PIN
                case Input_LimitC:
                case Input_LimitC_Max:
                    signal->mode.pull_mode = settings->limits.disable_pullup.b ? PullMode_Down : PullMode_Up;
                    signal->mode.irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef MPG_MODE_PIN
                case Input_MPGSelect:
                    signal->mode.pull_mode = PullMode_Up;
                    break;
#endif
#if I2C_STROBE_ENABLE
                case Input_KeypadStrobe:
                    signal->mode.pull_mode = PullMode_Up;
                    signal->mode.irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef SPINDLE_INDEX_PIN
                case Input_SpindleIndex:
                    signal->mode.pull_mode = PullMode_Down;
                    signal->mode.irq_mode = IRQ_Mode_Rising;
                    break;
#endif
#if QEI_ENABLE
                case Input_QEI_A:
                    if(qei_enable)
                        signal->mode.irq_mode = IRQ_Mode_Change;
                    break;

                case Input_QEI_B:
                    if(qei_enable)
                        signal->mode.irq_mode = IRQ_Mode_Change;
                    break;

  #if QEI_INDEX_ENABLED
                case Input_QEI_Index:
                    if(qei_enable)
                        signal->mode.irq_mode = IRQ_Mode_None;
                    break;
  #endif

  #if QEI_SELECT_ENABLED
                case Input_QEI_Select:
                    signal->mode.pull_mode = PullMode_Up;
                    signal->mode.debounce = hal.driver_cap.software_debounce;
                    if(qei_enable)
                        signal->mode.irq_mode = IRQ_Mode_Falling;
                    break;
  #endif
#endif
                default:
                    break;
            }

            signal->mode.debounce = hal.driver_cap.software_debounce && (signal->mode.debounce || signal->group == PinGroup_Control);

            pinMode(signal->pin, signal->mode.pull_mode == PullMode_Up ? INPUT_PULLUP : INPUT_PULLDOWN);

            if(signal->gpio.reg == (gpio_reg_t *)&GPIO6_DR)
                signal->offset = 0;
            else if(signal->gpio.reg == (gpio_reg_t *)&GPIO7_DR)
                signal->offset = 1;
            else if(signal->gpio.reg == (gpio_reg_t *)&GPIO8_DR)
                signal->offset = 2;
            else
                signal->offset = 3;

            if(signal->port != NULL)
                memcpy(signal->port, &signal->gpio, sizeof(gpio_t));

            if(signal->mode.irq_mode != IRQ_Mode_None) {

                pinEnableIRQ(signal, signal->mode.irq_mode);

                signal->active = (signal->gpio.reg->DR & signal->gpio.bit) != 0;

                if(signal->mode.irq_mode != IRQ_Mode_Change)
                    signal->active = signal->active ^ (signal->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1);
            }
        } while(i);

#if AUX_CONTROLS_ENABLED
        aux_ctrl_irq_enable(settings, aux_irq_handler);
#endif

        NVIC_ENABLE_IRQ(IRQ_GPIO6789);
    }
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    static xbar_t pin = {0};

    uint32_t i, id = 0;

    pin.mode.input = On;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {
        pin.id = id++;
        pin.pin = inputpin[i].pin;
        pin.function = inputpin[i].id;
        pin.group = inputpin[i].group;
        pin.mode.pwm = pin.group == PinGroup_SpindlePWM;
        pin.description = inputpin[i].description;

        pin_info(&pin, data);
    };

    pin.mode.mask = 0;
    pin.mode.output = On;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        pin.id = id++;
        pin.pin = outputpin[i].pin;
        pin.function = outputpin[i].id;
        pin.group = outputpin[i].group;
        pin.description = outputpin[i].description;

        pin_info(&pin, data);
    };

    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        pin.id = id++;
        pin.pin = ppin->pin.pin;
        pin.function = ppin->pin.function;
        pin.group = ppin->pin.group;
        pin.description = ppin->pin.description;

        pin_info(&pin, data);
    } while((ppin = ppin->next));
}

void registerPeriphPin (const periph_pin_t *pin)
{
    periph_signal_t *add_pin = malloc(sizeof(periph_signal_t));

    if(!add_pin)
        return;

    memcpy(&add_pin->pin, pin, sizeof(periph_pin_t));
    add_pin->next = NULL;

    if(periph_pins == NULL) {
        periph_pins = add_pin;
    } else {
        periph_signal_t *last = periph_pins;
        while(last->next)
            last = last->next;
        last->next = add_pin;
    }
}

void setPeriphPinDescription (const pin_function_t function, const pin_group_t group, const char *description)
{
    periph_signal_t *ppin = periph_pins;

    if(ppin) do {
        if(ppin->pin.function == function && ppin->pin.group == group) {
            ppin->pin.description = description;
            ppin = NULL;
        } else
            ppin = ppin->next;
    } while(ppin);
}

void pinModeOutput (gpio_t *gpio, uint8_t pin)
{
    pinMode(pin, OUTPUT);
    gpio->reg = (gpio_reg_t *)digital_pin_to_info_PGM[pin].reg;
    gpio->bit = digital_pin_to_info_PGM[pin].mask;
}

void pinEnableIRQ (const input_signal_t *signal, pin_irq_mode_t irq_mode)
{
    if(irq_mode == IRQ_Mode_None)
        signal->gpio.reg->IMR &= ~signal->gpio.bit; // Disable interrupt
    else if(irq_mode == IRQ_Mode_Change)
        signal->gpio.reg->EDGE_SEL |= signal->gpio.bit;
    else {
        uint32_t iopin = __builtin_ctz(signal->gpio.bit), shift, mode = 0;

        switch(irq_mode) {
            case IRQ_Mode_Rising:
                mode = 0b10;
                break;
            case IRQ_Mode_Falling:
                mode = 0b11;
                break;
            case IRQ_Mode_High:
                mode = 0b10;
                break;
            default: // Low
                mode = 0b00;
                break;
        }
        signal->gpio.reg->EDGE_SEL &= ~signal->gpio.bit;
        if(iopin < 16) {
           shift = iopin << 1;
           signal->gpio.reg->ICR1 = (signal->gpio.reg->ICR1 & ~(0b11 << shift)) | (mode << shift);
        } else {
           shift = (iopin - 16) << 1;
           signal->gpio.reg->ICR2 = (signal->gpio.reg->ICR2 & ~(0b11 << shift)) | (mode << shift);
        }
    }

    signal->gpio.reg->ISR = signal->gpio.bit;       // Clear interrupt.

    if(!(irq_mode == IRQ_Mode_None || (signal->group & (PinGroup_Limit|PinGroup_LimitMax))))    // If pin is not a limit pin
        signal->gpio.reg->IMR |= signal->gpio.bit;                                              // enable interrupt
}

#if QEI_ENABLE

static void qei_raise_event (void *data)
{
    hal.encoder.on_event(&qei.encoder, qei.count);
}

static void qei_update (void)
{
    const uint8_t encoder_valid_state[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

    uint_fast8_t idx;
    qei_state_t state = {0};

    state.a = (QEI_A.reg->DR & QEI_A.bit) != 0;
    state.b = (QEI_B.reg->DR & QEI_B.bit) != 0;

    idx = (((qei.state << 2) & 0x0F) | state.pins);

    if(encoder_valid_state[idx] ) {

        qei.state = ((qei.state << 4) | idx) & 0xFF;

        if (qei.state == 0x42 || qei.state == 0xD4 || qei.state == 0x2B || qei.state == 0xBD) {
            qei.count--;
            if(qei.vel_timeout == 0) {
                qei.encoder.event.position_changed = hal.encoder.on_event != NULL;
                protocol_enqueue_foreground_task(qei_raise_event, NULL);
            }
        } else if(qei.state == 0x81 || qei.state == 0x17 || qei.state == 0xE8 || qei.state == 0x7E) {
            qei.count++;
            if(qei.vel_timeout == 0) {
                qei.encoder.event.position_changed = hal.encoder.on_event != NULL;
                protocol_enqueue_foreground_task(qei_raise_event, NULL);
            }
        }
    }

}

static void qei_reset (uint_fast8_t id)
{
    qei.vel_timeout = 0;
    qei.count = qei.vel_count = 0;
    qei.vel_timestamp = millis();
    qei.vel_timeout = qei.encoder.axis != 0xFF ? QEI_VELOCITY_TIMEOUT : 0;
}

// dummy handler, called on events if plugin init fails
static void encoder_event (encoder_t *encoder, int32_t position)
{
    UNUSED(position);
    encoder->event.events = 0;
}

#endif

#if SDCARD_ENABLE

static char *sdcard_mount (FATFS **fs)
{
    static FATFS fatfs;
    static const char *dev = "1:/";

    if(f_mount(&fatfs, dev, 1) == FR_OK && f_chdrive(dev) == FR_OK)
        *fs = &fatfs;

    return (char *)dev;
}

static bool sdcard_unmount (FATFS **fs)
{
    return false; // for now
}

#endif

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
#if TRINAMIC_ENABLE && defined(BOARD_CNC_BOOSTERPACK) // Trinamic BoosterPack does not support mixed drivers
    driver_settings.trinamic.driver_enable.mask = AXES_BITMASK;
#endif

    /*************************
     *  Output signals init  *
     *************************/

    uint32_t i;
    for(i = 0 ; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        if(outputpin[i].group != PinGroup_AuxOutputAnalog)
            pinModeOutput(outputpin[i].port, outputpin[i].pin);
    }

    /******************
     *  Stepper init  *
     ******************/

    PIT_MCR = 0x00;
    CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);

    attachInterruptVector(IRQ_PIT, stepper_driver_isr);
    NVIC_SET_PRIORITY(IRQ_PIT, 2);
    NVIC_ENABLE_IRQ(IRQ_PIT);

    PULSE_TIMER_ENABLE &= ~(1 << 0);
    PULSE_TIMER_LOAD = 0;
    PULSE_TIMER_CTRL = TMR_CTRL_PCS(0b1000) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
    PULSE_TIMER_CSCTRL = TMR_CSCTRL_TCF1EN;

    attachInterruptVector(PULSE_TIMER_IRQ, stepper_pulse_isr);
    NVIC_SET_PRIORITY(PULSE_TIMER_IRQ, 0);
    NVIC_ENABLE_IRQ(PULSE_TIMER_IRQ);

    PULSE_TIMER_ENABLE |= (1 << 0);

#if STEP_INJECT_ENABLE
    PULSE2_TIMER_ENABLE &= ~(1 << 0);
    PULSE2_TIMER_LOAD = 0;
    PULSE2_TIMER_CTRL = TMR_CTRL_PCS(0b1000) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
    PULSE2_TIMER_CSCTRL = TMR_CSCTRL_TCF1EN;

    attachInterruptVector(PULSE2_TIMER_IRQ, output_pulse_isr);
    NVIC_SET_PRIORITY(PULSE2_TIMER_IRQ, 0);
    NVIC_ENABLE_IRQ(PULSE2_TIMER_IRQ);

    PULSE2_TIMER_ENABLE |= (1 << 0);
#endif

   /***********************
    *  Control pins init  *
    ***********************/

    attachInterruptVector(IRQ_GPIO6789, gpio_isr);

#if DRIVER_SPINDLE_PWM_ENABLE

   /******************
    *  Spindle init  *
    ******************/

    SPINDLE_PWM_TIMER_ENABLE &= (1 << SPINDLE_PWM_TIMER_C);
    SPINDLE_PWM_TIMER_LOAD = 0;
    SPINDLE_PWM_TIMER_CTRL = TMR_CTRL_PCS(0b1001) | TMR_CTRL_OUTMODE(0b100) | TMR_CTRL_LENGTH;
    SPINDLE_PWM_TIMER_SCTRL = TMR_SCTRL_OEN | TMR_SCTRL_FORCE;
    SPINDLE_PWM_TIMER_ENABLE |= (1 << SPINDLE_PWM_TIMER_C);

    *(portConfigRegister(SPINDLE_PWM_PIN)) = 1;

    static const periph_pin_t pwm = {
        .function = Output_SpindlePWM,
        .group = PinGroup_SpindlePWM,
        .pin = SPINDLE_PWM_PIN,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    hal.periph_port.register_pin(&pwm);

#if PPI_ENABLE

    PPI_TIMER_ENABLE = 0;
    PPI_TIMER_LOAD = 0;
    PPI_TIMER_COMP1 = (uint16_t)((1500UL * F_BUS_MHZ) / 128);
    PPI_TIMER_CTRL = TMR_CTRL_PCS(0b1111) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
    PPI_TIMER_CSCTRL = TMR_CSCTRL_TCF1EN;

    attachInterruptVector(PPI_TIMER_IRQ, ppi_timeout_isr);
    NVIC_SET_PRIORITY(PPI_TIMER_IRQ, 3);
    NVIC_ENABLE_IRQ(PPI_TIMER_IRQ);

    PPI_TIMER_ENABLE = 1;

    ppi_init();

#endif // PPI_ENABLE

#endif // DRIVER_SPINDLE_PWM_ENABLE

#if SPINDLE_ENCODER_ENABLE

    CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
    CCM_CMEOR |= CCM_CMEOR_MOD_EN_OV_GPT;

    // Free running timer
    GPT1_CR = 0;
    GPT1_CR |= GPT_CR_SWR;
    while(GPT1_CR & GPT_CR_SWR);
    GPT1_CR = GPT_CR_CLKSRC(1);
    GPT1_CR |= GPT_CR_FRR|GPT_CR_ENMOD|GPT_CR_EN;
    GPT1_PR = 150;

  #if SPINDLE_PULSE_PIN == 14

    CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
//    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x001d0b0;
    IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

    // Spindle pulse counter
    GPT2_CR = 0;
    GPT2_CR |= GPT_CR_SWR;
    while(GPT2_CR & GPT_CR_SWR);
    GPT2_CR = GPT_CR_CLKSRC(3);
    GPT2_CR |= GPT_CR_ENMOD;
    GPT2_CR |= GPT_CR_FRR|GPT_CR_EN;
    GPT2_OCR1 = spindle_encoder.tics_per_irq;
    GPT2_IR = GPT_IR_OF1IE;

    static const periph_pin_t spp = {
        .function = Input_SpindlePulse,
        .group = PinGroup_SpindlePulse,
        .pin = SPINDLE_PULSE_PIN,
        .mode = { .input = On }
    };

    hal.periph_port.register_pin(&spp);

    attachInterruptVector(IRQ_GPT2, spindle_pulse_isr);
    NVIC_SET_PRIORITY(IRQ_GPT2, 1);
    NVIC_ENABLE_IRQ(IRQ_GPT2);

  #endif

#endif // SPINDLE_ENCODER_ENABLE

  // Set defaults

    IOInitDone = settings->version == 22;

    hal.settings_changed(settings, (settings_changed_flags_t){0});
    hal.stepper.go_idle(true);

#if IOPORTS_ENABLE
    ioports_init();
#endif

#if SDCARD_ENABLE
    sdcard_events_t *card = sdcard_init();
    card->on_mount = sdcard_mount;
    card->on_unmount = sdcard_unmount;
#endif

#if LITTLEFS_ENABLE
    fs_littlefs_mount("/littlefs", t4_littlefs_hal());
#endif

#if ETHERNET_ENABLE
    grbl_enet_start();
#endif

#if QEI_ENABLE
    if(qei_enable)
        encoder_start(&qei.encoder);
#endif

    return IOInitDone;
}

#if EEPROM_ENABLE == 0

// EEPROM emulation - stores settings in flash

bool nvsRead (uint8_t *dest)
{
// assert size ? E2END

    eeprom_read_block(dest, 0, hal.nvs.size);

    return true; //?;
}

bool nvsWrite (uint8_t *source)
{
    eeprom_write_block(source, 0, hal.nvs.size);

    return true; //?;
}

// End EEPROM emulation

#endif

#ifdef DEBUGOUT

void debugOut (bool on)
{
    digitalWrite(13, on); // LED
}

#endif

// Cold restart (T4.x has no reset button)
static void reboot (void)
{
    SCB_AIRCR = 0x05FA0004;
}

static bool set_rtc_time (struct tm *time)
{
    rtc_started = true;

    time_t t = mktime(time);

    rtc_set(t);

    return true;
}

static bool get_rtc_time (struct tm *time)
{
    if(rtc_started) {
        time_t t = rtc_get();
        struct tm *dt = gmtime(&t);
        memcpy(time, dt, sizeof(struct tm));
    }

    return rtc_started;
}

// https://forum.pjrc.com/threads/33443-How-to-display-free-ram?highlight=free+memory
extern char _heap_end[], *__brkval;

// This should ideall return sum of all free blocks on the heap...
uint32_t get_free_mem (void)
{
    return _heap_end - __brkval;
}

inline static uint64_t get_micros (void)
{
    return (uint64_t)micros();
}

// Initialize HAL pointers, setup serial comms and enable EEPROM.
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done.
bool driver_init (void)
{
    static char options[30];

    uint32_t i;

    // Chain our systick isr to the Arduino handler

    if(systick_isr_org == NULL)
        systick_isr_org = _VectorsRam[15];
    _VectorsRam[15] = systick_isr;

    // Enable lazy stacking of FPU registers here if a FPU is available.

 //   FPU->FPCCR = (FPU->FPCCR & ~FPU_FPCCR_LSPEN_Msk) | FPU_FPCCR_ASPEN_Msk;  // enable lazy stacking

#ifdef MPG_MODE_PIN
    // Pull down MPG mode pin until startup is completed.
    i = 0;
    while(mpg_pin == NULL) {
        if(inputpin[i].pin == MPG_MODE_PIN) {
            mpg_pin = &inputpin[i];
            pinModeOutput(mpg_pin->port, mpg_pin->pin);
            DIGITAL_OUT(mpg_pin->gpio, 0);
        }
        i++;
    }
#endif

    options[0] = '\0';

#if USB_SERIAL_CDC == 1
    strcat(options, "USB.1 ");
#endif
#if USB_SERIAL_CDC == 2
    strcat(options, "USB.2 ");
#endif

    if(*options != '\0')
        options[strlen(options) - 1] = '\0';

    hal.info = "iMXRT1062";
    hal.driver_version = "240928";
    hal.driver_url = GRBL_URL "/iMXRT1062";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
#ifdef BOARD_URL
    hal.board_url = BOARD_URL;
#endif

    hal.driver_options = *options == '\0' ? NULL : options;
    hal.driver_setup = driver_setup;
    hal.f_mcu = F_CPU_ACTUAL / 1000000UL;
    hal.f_step_timer = 24000000;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.get_free_mem = get_free_mem;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#ifdef GANGING_ENABLED
    hal.stepper.get_ganged = getGangedAxes;
#endif
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif
#if STEP_INJECT_ENABLE
    hal.stepper.output_step = stepperOutputStep;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

#if PROBE_ENABLE
    hal.probe.configure = probeConfigure;
    hal.probe.get_state = probeGetState;
#endif

    hal.control.get_state = systemGetState;

    hal.reboot = reboot;
    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
#if I2C_STROBE_ENABLE
    hal.irq_claim = irq_claim;
#endif
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_micros = get_micros;
    hal.get_elapsed_ticks = millis;
    hal.enumerate_pins = enumeratePins;
    hal.periph_port.register_pin = registerPeriphPin;
    hal.periph_port.set_pin_description = setPeriphPinDescription;

    hal.rtc.get_datetime = get_rtc_time;
    hal.rtc.set_datetime = set_rtc_time;

    serialRegisterStreams();

#if USB_SERIAL_CDC
    const io_stream_t *st = usb_serialInit();
    stream_connect(st);
#else
    if(!stream_connect_instance(SERIAL_STREAM, BAUD_RATE))
        while(true); // Cannot boot if no communication channel is available!
#endif

#ifdef I2C_PORT
    i2c_init();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else // use Arduino emulated EEPROM in flash
    eeprom_initialize();
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = nvsRead;
    hal.nvs.memcpy_to_flash = nvsWrite;
#endif

#if QEI_ENABLE
    hal.encoder.reset = qei_reset;
    hal.encoder.on_event = encoder_event;
#endif

#if DRIVER_SPINDLE_ENABLE

 #if DRIVER_SPINDLE_PWM_ENABLE

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_PWM,
#if DRIVER_SPINDLE_DIR_ENABLE
        .ref_id = SPINDLE_PWM0,
#else
        .ref_id = SPINDLE_PWM0_NODIR,
#endif
        .config = spindleConfig,
        .set_state = spindleSetStateVariable,
        .get_state = spindleGetState,
        .get_pwm = spindleGetPWM,
        .update_pwm = spindleSetSpeed,
  #if PPI_ENABLE
        .pulse_on = spindlePulseOn,
  #endif
        .cap = {
            .gpio_controlled = On,
            .variable = On,
            .laser = On,
            .pwm_invert = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #else

    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Basic,
#if DRIVER_SPINDLE_DIR_ENABLE
        .ref_id = SPINDLE_ONOFF0_DIR,
#else
        .ref_id = SPINDLE_ONOFF0,
#endif
        .set_state = spindleSetState,
        .get_state = spindleGetState,
        .cap = {
            .gpio_controlled = On,
  #if DRIVER_SPINDLE_DIR_ENABLE
            .direction = On
  #endif
        }
    };

 #endif

    spindle_id = spindle_register(&spindle, DRIVER_SPINDLE_NAME);

#endif // DRIVER_SPINDLE_ENABLE

// Driver capabilities
// See driver_cap_t union i grbl/hal.h for available flags.

#if ESTOP_ENABLE
    hal.signals_cap.reset = Off;
    hal.signals_cap.e_stop = On;
#endif
#if SAFETY_DOOR_BIT
    hal.signals_cap.safety_door_ajar = On;
#endif
#if LIMITS_OVERRIDE_BIT
    hal.signals_cap.limits_override = On;
#endif
    hal.limits_cap = get_limits_cap();
    hal.home_cap = get_home_cap();
#ifdef COOLANT_FLOOD_PIN
    hal.coolant_cap.flood = On;
#endif
#ifdef COOLANT_MIST_PIN
    hal.coolant_cap.mist = On;
#endif
#if SPINDLE_ENCODER_ENABLE
    hal.driver_cap.spindle_encoder = On;
#endif
#if SPINDLE_SYNC_ENABLE
    hal.driver_cap.spindle_sync = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

    static pin_group_pins_t aux_digital_in = {0}, aux_digital_out = {0}, aux_analog_in = {0}, aux_analog_out = {0};

    input_signal_t *input;

    for(i = 0; i < sizeof(inputpin) / sizeof(input_signal_t); i++) {

        input = &inputpin[i];
        input->mode.input = input->cap.input = On;
        input->gpio.reg = (gpio_reg_t *)digital_pin_to_info_PGM[input->pin].reg;
        input->gpio.bit = digital_pin_to_info_PGM[input->pin].mask;

        if(input->group == PinGroup_AuxInput) {
            if(aux_digital_in.pins.inputs == NULL)
                aux_digital_in.pins.inputs = input;
            input->user_port = aux_digital_in.n_pins++;
            input->id = (pin_function_t)(Input_Aux0 + input->user_port);
            input->cap.irq_mode = IRQ_Mode_All;
            input->mode.pull_mode = PullMode_Up;
            input->cap.irq_mode = IRQ_Mode_All;
            input->cap.pull_mode = PullMode_UpDown;
            input->cap.debounce = hal.driver_cap.software_debounce;
#if AUX_CONTROLS_ENABLED
            aux_ctrl_t *aux_remap;
            if((aux_remap = aux_ctrl_remap_explicit(NULL, input->pin, input->user_port, input))) {
                if(aux_remap->function == Input_Probe)
                    aux_remap->irq_mode = IRQ_Mode_Change;
            }
#endif
        } else if(input->group & (PinGroup_Limit|PinGroup_LimitMax)) {
            if(limit_inputs.pins.inputs == NULL)
                limit_inputs.pins.inputs = input;
            limit_inputs.n_pins++;
        }
    }

    output_signal_t *output;

    for(i = 0; i < sizeof(outputpin) / sizeof(output_signal_t); i++) {
        output = &outputpin[i];
        output->mode.output = On;
        if(output->group == PinGroup_AuxOutput) {
            if(aux_digital_out.pins.outputs == NULL)
                aux_digital_out.pins.outputs = output;
            output->id = (pin_function_t)(Output_Aux0 + aux_digital_out.n_pins++);
        } else if(output->group == PinGroup_AuxOutputAnalog) {
            if(aux_analog_out.pins.outputs == NULL)
                aux_analog_out.pins.outputs = output;
            output->mode.analog = On;
            output->id = (pin_function_t)(Output_Analog_Aux0 + aux_analog_out.n_pins++);
        }
    }

    ioports_init(&aux_digital_in, &aux_digital_out);

    if(aux_analog_out.n_pins)
        ioports_init_analog(&aux_analog_in, &aux_analog_out);

#if defined(SAFETY_DOOR_PIN)
//    hal.signals_cap.safety_door = On;
#endif

#if AUX_CONTROLS_ENABLED
    aux_ctrl_claim_ports(aux_claim_explicit, NULL);
#endif

#if ETHERNET_ENABLE
    grbl_enet_init();
#endif

#if QEI_ENABLE
    qei_enable = encoder_init(QEI_ENABLE);
#endif

#ifdef NEOPIXEL_UART_PIN
    extern void neopixel_init (void);
    neopixel_init();
#endif

#include "grbl/plugins_init.h"

#if MPG_ENABLE == 1
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, NULL);
    if(hal.driver_cap.mpg_mode)
        protocol_enqueue_foreground_task(mpg_enable, NULL);
#elif MPG_ENABLE == 2
    if(!hal.driver_cap.mpg_mode)
        hal.driver_cap.mpg_mode = stream_mpg_register(stream_open_instance(MPG_STREAM, 115200, NULL, NULL), false, stream_mpg_check_enable);
#endif

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 10;
}

/* interrupt handlers */

// Main stepper driver.
static void stepper_driver_isr (void)
{
    if(PIT_TFLG0 & PIT_TFLG_TIF) {
        PIT_TFLG0 |= PIT_TFLG_TIF;
        hal.stepper.interrupt_callback();
    }
}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/
// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
static void stepper_pulse_isr (void)
{
    PULSE_TIMER_CSCTRL &= ~TMR_CSCTRL_TCF1;

    set_step_outputs((axes_signals_t){0});
}

static void stepper_pulse_isr_delayed (void)
{
    PULSE_TIMER_CSCTRL &= ~TMR_CSCTRL_TCF1;

    set_step_outputs(next_step_outbits);

    attachInterruptVector(PULSE_TIMER_IRQ, stepper_pulse_isr);
    PULSE_TIMER_COMP1 = pulse_length;
    PULSE_TIMER_CTRL |= TMR_CTRL_CM(0b001);
}

#if STEP_INJECT_ENABLE

static void output_pulse_isr (void)
{
    PULSE2_TIMER_CSCTRL &= ~TMR_CSCTRL_TCF1;

    stepperInjectStep(settings.steppers.step_invert);
    pulse_output.mask = 0;
}

static void output_pulse_isr_delayed (void)
{
    PULSE2_TIMER_CSCTRL &= ~TMR_CSCTRL_TCF1;

    axes_signals_t step_outbits;
    step_outbits.value =  pulse_output.value ^ settings.steppers.step_invert.mask;
    stepperInjectStep(step_outbits);

    attachInterruptVector(PULSE2_TIMER_IRQ, output_pulse_isr);
    PULSE2_TIMER_COMP1 = pulse_length;
    PULSE2_TIMER_CTRL |= TMR_CTRL_CM(0b001);
}

#endif // STEP_INJECT_ENABLE

#if SPINDLE_ENCODER_ENABLE && SPINDLE_PULSE_PIN == 14

static void spindle_pulse_isr (void)
{
    uint32_t tval = GPT1_CNT;

    GPT2_SR |= GPT_SR_OF1; // clear interrupt flag
    GPT2_OCR1 += spindle_encoder.tics_per_irq;

    spindleLock = true;

    spindle_encoder.counter.pulse_count = GPT2_CNT;
    spindle_encoder.counter.last_count = spindle_encoder.counter.pulse_count;
    spindle_encoder.timer.pulse_length = tval - spindle_encoder.timer.last_pulse;
    spindle_encoder.timer.last_pulse = tval;

    spindleLock = false;
}

#endif // SPINDLE_ENCODER_ENABLE && SPINDLE_PULSE_PIN == 14


#if PPI_ENABLE
// Switches off the spindle (laser) after laser.pulse_length time has elapsed
static void ppi_timeout_isr (void)
{
    PPI_TIMER_CSCTRL &= ~TMR_CSCTRL_TCF1;
    spindle_off();
}
#endif

void pin_debounce (void *pin)
{
    input_signal_t *input = (input_signal_t *)pin;

#if SAFETY_DOOR_ENABLE
    if(input->id == Input_SafetyDoor)
        debounce.safety_door = Off;
#endif
#ifdef QEI_SELECT_PIN
    if(input->id == Input_QEI_Select)
        debounce.qei_select = Off;
#endif

    if(input->mode.irq_mode == IRQ_Mode_Change ||
        !!(input->gpio.reg->DR & input->gpio.bit) == (input->mode.irq_mode == IRQ_Mode_Falling ? 0 : 1)) {

        switch(input->group) {

            case PinGroup_Limit:
            case PinGroup_LimitMax:
                {
                    limit_signals_t state = limitsGetState();
                    if(limit_signals_merge(state).value)
                        hal.limits.interrupt_callback(state);
                }
                break;

            case PinGroup_Control:
                hal.control.interrupt_callback(systemGetState());
                break;

            case PinGroup_AuxInput:
                ioports_event(input);
                break;

#ifdef QEI_SELECT_PIN
            case PinGroup_QEI_Select:
                qei_select_handler();
                break;
#endif
            default:
                break;
        }
    }

    input->gpio.reg->IMR |= input->gpio.bit; // Reenable pin interrupt
}

  //GPIO intr process
static void gpio_isr (void)
{
    uint32_t grp = 0, intr_status[4];

    // Get masked interrupt status
    intr_status[0] = ((gpio_reg_t *)&GPIO6_DR)->ISR & ((gpio_reg_t *)&GPIO6_DR)->IMR;
    intr_status[1] = ((gpio_reg_t *)&GPIO7_DR)->ISR & ((gpio_reg_t *)&GPIO7_DR)->IMR;
    intr_status[2] = ((gpio_reg_t *)&GPIO8_DR)->ISR & ((gpio_reg_t *)&GPIO8_DR)->IMR;
    intr_status[3] = ((gpio_reg_t *)&GPIO9_DR)->ISR & ((gpio_reg_t *)&GPIO9_DR)->IMR;

    // Clear interrupts
    ((gpio_reg_t *)&GPIO6_DR)->ISR = intr_status[0];
    ((gpio_reg_t *)&GPIO7_DR)->ISR = intr_status[1];
    ((gpio_reg_t *)&GPIO8_DR)->ISR = intr_status[2];
    ((gpio_reg_t *)&GPIO9_DR)->ISR = intr_status[3];

    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
    do {
        if(inputpin[--i].mode.irq_mode != IRQ_Mode_None || inputpin[i].group == PinGroup_AuxInput) {

            if(intr_status[inputpin[i].offset] & inputpin[i].gpio.bit) {

                inputpin[i].active = true;

                if(inputpin[i].mode.debounce && task_add_delayed(pin_debounce, &inputpin[i], 40)) {
                    inputpin[i].gpio.reg->IMR &= ~inputpin[i].gpio.bit; // Disable pin interrupt
                    if(inputpin[i].id == Input_SafetyDoor)
                        debounce.safety_door = On;
                    else if(inputpin[i].id == Input_QEI_Select)
                        debounce.qei_select = On;
                }  else switch(inputpin[i].group) {
#if QEI_ENABLE
                    case PinGroup_QEI:
                        qei_update();
                        break;
#endif

#if SPINDLE_ENCODER_ENABLE && defined(SPINDLE_INDEX_PIN)
                    case PinGroup_SpindleIndex:
                        spindleLock = true;
                        spindle_encoder.counter.index_count++;
                        spindle_encoder.counter.last_index = GPT2_CNT;
                        spindle_encoder.timer.last_index = GPT1_CNT;
                        spindleLock = false;
                        break;
#endif

#if I2C_STROBE_ENABLE && !defined(AUX_DEVICES)
                    case PinGroup_Keypad:
                        if(i2c_strobe.callback)
                            i2c_strobe.callback(0, !(KeypadStrobe.reg->DR & KeypadStrobe.bit));
                        break;
#endif

                    case PinGroup_AuxInput:
                        ioports_event(&inputpin[i]);
                        break;

#if MPG_ENABLE == 1
                    case PinGroup_MPG:
                        pinEnableIRQ(&inputpin[i], IRQ_Mode_None);
                        protocol_enqueue_foreground_task(mpg_select, NULL);
                        break;
#endif
                    default:
                        grp |= inputpin[i].group;
                        break;
                }
            }
        }
    } while(i);

    if(grp & (PinGroup_Limit|PinGroup_LimitMax)) {
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }

    if(grp & PinGroup_Control)
        hal.control.interrupt_callback(systemGetState());

#if QEI_SELECT_ENABLED
    if(grp & PinGroup_QEI_Select)
        qei_select_handler
#endif
}

// Interrupt handler for 1 ms interval timer
static void systick_isr (void)
{
    systick_isr_org();

#if QEI_ENABLE
      if(qei.vel_timeout && !(--qei.vel_timeout)) {
          qei.encoder.velocity = abs(qei.count - qei.vel_count) * 1000 / (millis() - qei.vel_timestamp);
          qei.vel_timestamp = millis();
          qei.vel_timeout = QEI_VELOCITY_TIMEOUT;
          if((qei.encoder.event.position_changed = !qei.dbl_click_timeout || qei.encoder.velocity == 0))
              protocol_enqueue_foreground_task(qei_raise_event, NULL);
          qei.vel_count = qei.count;
      }

      if(qei.dbl_click_timeout && !(--qei.dbl_click_timeout)) {
          qei.encoder.event.click = On;
          protocol_enqueue_foreground_task(qei_raise_event, NULL);
      }
#endif

    if(grbl_delay.ms && !(--grbl_delay.ms)) {
        if(grbl_delay.callback) {
            grbl_delay.callback();
            grbl_delay.callback = NULL;
        }
    }
}
