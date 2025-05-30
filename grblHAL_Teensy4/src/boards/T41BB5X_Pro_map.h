/*
  T41BB5X_Pro_map.h - driver code for IMXRT1062 processor (on Teensy 4.1 board)

  Part of grblHAL

  Board by Phil Barrett: https://github.com/phil-barrett/grblHAL-teensy-4.x

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

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#define BOARD_NAME "T41BB5X Pro"
#define BOARD_URL "https://github.com/phil-barrett/grbl-teensy-4"

#if N_AXIS > 5
#error Max number of axes is 5 for T41U5XBB
#endif

#if QEI_ENABLE && SPINDLE_SYNC_ENABLE
#error Quadrature encoder and spindle sync cannot be enabled at the same time
#endif

// Board has 2K FRAM
/* supply problems...
#undef EEPROM_ENABLE
#undef EEPROM_IS_FRAM
#define EEPROM_ENABLE   1
#define EEPROM_IS_FRAM  1
*/
#define X_STEP_PIN          (2u)
#define X_DIRECTION_PIN     (3u)
#define X_ENABLE_PIN        (10u)
#define X_LIMIT_PIN         (20u)

#define Y_STEP_PIN          (4u)
#define Y_DIRECTION_PIN     (5u)
#define Y_ENABLE_PIN        (35u)
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
#define M3_ENABLE_PIN       (38u)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PIN         (26u)
#define M4_DIRECTION_PIN    (27u)
#define M4_LIMIT_PIN        (28u)
#define M4_ENABLE_PIN       (37u)
#endif

// Define auxiliary output pins
#define AUXOUTPUT0_PIN      (34U)
#define AUXOUTPUT1_PIN      (32U)
#if SPINDLE_ENABLE & (1<<SPINDLE_PWM2|1<<SPINDLE_PWM2_NODIR)
#define AUXOUTPUT0_PWM_PIN  (33U)
#else
#define AUXOUTPUT2_PIN      (33U)
#endif
#define AUXOUTPUT3_PIN      (12u) // Spindle enable
#define AUXOUTPUT4_PIN      (11u) // Spindle direction
#define AUXOUTPUT5_PIN      (13u) // Spindle PWM
#define AUXOUTPUT6_PIN      (19u) // Coolant flood
#define AUXOUTPUT7_PIN      (18u) // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT3_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT5_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT4_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT6_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT7_PIN
#endif

// Define auxiliary input pins
#if !QEI_ENABLE
#define AUXINPUT0_PIN       (36u) // ST0
#define AUXINPUT1_PIN       (30u) // ST1
#endif
#if !SPINDLE_SYNC_ENABLE
#define AUXINPUT2_PIN       (31u) // ST2
#define AUXINPUT3_PIN       (41u) // ST3
#endif
#define AUXINPUT4_PIN       (41u) // I2C strobe
#if !defined(M4_LIMIT_PIN)
#define AUXINPUT5_PIN       (28u) // MPG mode
#endif
#define AUXINPUT6_PIN       (29u) // Safety door
#define AUXINPUT7_PIN       (15u) // Probe
#define AUXINPUT8_PIN       (40u) // Reset/EStop
#define AUXINPUT9_PIN       (16u) // Feed hold
#define AUXINPUT10_PIN      (17u) // Cycle start

// Define user-control controls (cycle start, reset, feed hold) input pins.
#if CONTROL_ENABLE & CONTROL_HALT
#define RESET_PIN           AUXINPUT8_PIN
#endif
#if CONTROL_ENABLE & CONTROL_FEED_HOLD
#define FEED_HOLD_PIN       AUXINPUT9_PIN
#endif
#if CONTROL_ENABLE & CONTROL_CYCLE_START
#define CYCLE_START_PIN     AUXINPUT10_PIN
#endif

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT7_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT6_PIN
#endif

#if MPG_ENABLE == 1 && defined(AUXINPUT5_PIN)
#define MPG_MODE_PIN        AUXINPUT5_PIN
#endif

#if I2C_STROBE_ENABLE
#define I2C_STROBE_PIN      AUXINPUT4_PIN
#endif

#if MOTOR_FAULT_ENABLE && defined(AUXINPUT0_PIN)
#define MOTOR_FAULT_PIN     AUXINPUT0_PIN
#endif

#if MOTOR_WARNING_ENABLE && defined(AUXINPUT1_PIN)
#define MOTOR_WARNING_PIN   AUXINPUT1_PIN
#endif

#if QEI_ENABLE
#define QEI_A_PIN           (36u) // ST0
#define QEI_B_PIN           (30u) // ST1
#ifdef AUXINPUT2_PIN
#define QEI_SELECT_PIN      AUXINPUT2_PIN
#endif
#endif

#if SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_PIN   (31u) // ST2
#define SPINDLE_PULSE_PIN   (14u) // ST3
#endif


#if I2C_ENABLE
#define I2C_PORT    	4
#define I2C_SCL4    	(24u) // Not used, for info only
#define I2C_SDA4    	(25u) // Not used, for info only
#else
#define SERIAL1_PORT	1
#define UART1_RX    	(25u) // Not used, for info only
#define UART1_TX    	(24u) // Not used, for info only
#endif
