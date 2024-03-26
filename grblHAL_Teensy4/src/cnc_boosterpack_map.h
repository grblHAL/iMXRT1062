/*
  cnc_boosterpack_map.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

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

#define BOARD_NAME "CNC BoosterPack"
#define BOARD_URL "https://github.com/terjeio/CNC_Boosterpack"

#if N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#if SPINDLE_SYNC_ENABLE
#error "Spindle sync is not supported for CNC BoosterPack"
#endif

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif
#define EEPROM_ENABLE       16 // CNC BoosterPack has on-board EEPROM

#if I2C_ENABLE
#define I2C_PORT            0
#define I2C_SCL0            (19u) // Not referenced, for info only
#define I2C_SDA0            (18u) // Not referenced, for info only
#endif

#define UART_PORT           8
#define UART_RX5            (21u) // Not referenced, for info only
#define UART_TX5            (20u) // Not referenced, for info only

// Define step pulse output pins.
#define X_STEP_PIN          (32u)
#define Y_STEP_PIN          (30u)
#define Z_STEP_PIN          (26u)

// Define step direction output pins.
#define X_DIRECTION_PIN     (5u)
#define Y_DIRECTION_PIN     (33u)
#define Z_DIRECTION_PIN     (13u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN (24u)
#define Z_ENABLE_PIN        (8u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN         (10u)
#define Y_LIMIT_PIN         (1u)
#define Z_LIMIT_PIN         (0u)

// Define driver spindle pins

#if DRIVER_SPINDLE_PWM_ENABLE
#define SPINDLE_PWM_PIN         (12u)
#else
#define AUXOUTPUT0_PIN          (12u)
#endif

#if DRIVER_SPINDLE_DIR_ENABLE
#define SPINDLE_DIRECTION_PIN   (16u)
#else
#define AUXOUTPUT1_PIN          (16u)
#endif

#if DRIVER_SPINDLE_ENABLE
#define SPINDLE_ENABLE_PIN      (14u)
#else
#define AUXOUTPUT2_PIN          (14u)
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN   (4u)
#define COOLANT_MIST_PIN    (31u)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           (11u)
#define FEED_HOLD_PIN       (7u)
#define CYCLE_START_PIN     (6u)

#if !QEI_ENABLE
#define AUXINPUT0_PIN       (3u)
#endif
#define AUXINPUT1_PIN       (29u)
#define AUXINPUT2_PIN       (27u)
#if !QEI_ENABLE
#define AUXINPUT3_PIN       (2u)
#endif
#define AUXINPUT4_PIN       (28u) // I2C strobe
#define AUXINPUT5_PIN       (9u)  // Safety door
#define AUXINPUT6_PIN       (15u) // Probe

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT6_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT5_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN      AUXINPUT4_PIN
#endif

#if QEI_ENABLE
    #define QEI_A_PIN       (3u)
    #define QEI_B_PIN       (2u)
//    #define QEI_INDEX_PIN  GPIO2_PIN
    #define QEI_SELECT_PIN AUXINPUT1_PIN
#endif

/* EOF */
