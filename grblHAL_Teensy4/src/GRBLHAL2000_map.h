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

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PIN         (13u)
#else
#define AUXOUTPUT6_PIN          (13u)
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PIN   (11u)
#else
#define AUXOUTPUT5_PIN          (11u)
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PIN      (12u)
#else
#define AUXOUTPUT4_PIN          (12u)
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN   (19u)
#define COOLANT_MIST_PIN    (18u)

// Define auxillary input pins
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

// Define auxillary output pins
#define AUXOUTPUT0_PIN      (37u)
#define AUXOUTPUT1_PIN      (32u)
#define AUXOUTPUT2_PIN      (33u)
#define AUXOUTPUT3_PIN      (38u)

#if I2C_ENABLE
#define I2C_PORT            4
#define I2C_SCL4            (24u) // Not used, for info only
#define I2C_SDA4            (25u) // Not used, for info only
#endif
