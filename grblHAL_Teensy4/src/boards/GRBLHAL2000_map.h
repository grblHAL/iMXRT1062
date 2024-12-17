/*
  GRBLHAL2000_map.h - driver code for IMXRT1062 processor (on Teensy 4.1 board)

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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

#define BOARD_NAME "GRBLHAL2000 - PRINTNC"
#define BOARD_URL "https://github.com/Expatria-Technologies/grblhal_2000_PrintNC"
#define HAS_BOARD_INIT

#if MODBUS_ENABLE < 1
#define UART_PORT 5
#endif

#ifdef NETWORK_HOSTNAME
    #undef NETWORK_HOSTNAME
    #define NETWORK_HOSTNAME "GRBLHAL2000"
#endif

#if N_AXIS > 5
#error Max number of axes is 5 for UniversalCNC
#endif

#if QEI_ENABLE && SPINDLE_SYNC_ENABLE
#error Quadrature encoder and spindle sync cannot be enabled at the same time
#endif

#define X_STEP_PIN          (2u)
#define X_DIRECTION_PIN     (3u)
#define X_ENABLE_PIN        (10u)
#define X_LIMIT_PIN         (20u)

#define Y_STEP_PIN          (4u)
#define Y_DIRECTION_PIN     (5u)
#define Y_ENABLE_PIN        (10u)
#define Y_LIMIT_PIN         (21u)

#define Z_STEP_PIN          (6u)
#define Z_DIRECTION_PIN     (7u)
#define Z_ENABLE_PIN        (39u)
#define Z_LIMIT_PIN         (22u)

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         (8u)
#define M3_DIRECTION_PIN    (9u)
#define M3_LIMIT_PIN        (23u)
#define M3_ENABLE_PIN       (10u)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PIN         (26u)
#define M4_DIRECTION_PIN    (27u)
#define M4_LIMIT_PIN        (28u)
#define M4_ENABLE_PIN       (10u)
#endif

// Define auxiliary output pins
#define AUXOUTPUT0_PIN      (37u)
#define AUXOUTPUT1_PIN      (32u)
#define AUXOUTPUT2_PIN      (33u)
#define AUXOUTPUT3_PIN      (38u)
#define AUXOUTPUT4_PIN      (12u) // Spindle enable
#define AUXOUTPUT5_PIN      (11u) // Spindle direction
#define AUXOUTPUT6_PIN      (13u) // Spindle PWM
#define AUXOUTPUT7_PIN      (19u) // Coolant flood
#define AUXOUTPUT8_PIN      (18u) // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT4_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT6_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT5_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT7_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT8_PIN
#endif

// Define auxiliary input pins
#if !QEI_ENABLE
#define AUXINPUT0_PIN       (36u) // ST0
#define AUXINPUT1_PIN       (30u) // ST1
#endif
#if !SPINDLE_SYNC_ENABLE
#define AUXINPUT2_PIN       (31u) // ST2
#define AUXINPUT3_PIN       (14u) // ST3
#endif
#define AUXINPUT4_PIN       (29u) // Safety door
#define AUXINPUT5_PIN       (41u) // I2C strobe
#define AUXINPUT6_PIN       (15u) // Probe

// Define user-control CONTROLs (cycle start, reset, feed hold, door) input pins.
#define RESET_PIN           (40u)  //this is halt?
#define FEED_HOLD_PIN       (16u)
#define CYCLE_START_PIN     (17u)

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT6_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT4_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN     AUXINPUT4_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN      AUXINPUT5_PIN
#endif

#if QEI_ENABLE
#define QEI_A_PIN           (36u)
#define QEI_B_PIN           (30u)
#if defined(AUXINPUT2_PIN)
#define QEI_SELECT_PIN      AUXINPUT2_PIN
#endif
#endif

#if SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_PIN   (31u) // ST2
#define SPINDLE_PULSE_PIN   (14u) // ST3
#endif

#if I2C_ENABLE
#define I2C_PORT            4
#define I2C_SCL4            (24u) // Not used, for info only
#define I2C_SDA4            (25u) // Not used, for info only
#endif
