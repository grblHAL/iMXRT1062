/*
  generic_map.h - driver code for IMXRT1062 processor (on Teensy 4.x board)

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

#if N_ABC_MOTORS > 2
#error "Axis configuration is not supported!"
#endif

#if SPINDLE_SYNC_ENABLE
#error "Spindle sync is not supported"
#endif

// Define step pulse output pins.
#define X_STEP_PIN          (2u)
#define Y_STEP_PIN          (4u)
#define Z_STEP_PIN          (6u)

// Define step direction output pins.
#define X_DIRECTION_PIN     (3u)
#define Y_DIRECTION_PIN     (5u)
#define Z_DIRECTION_PIN     (7u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN (10u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN         (20u)
#define Y_LIMIT_PIN         (21u)
#define Z_LIMIT_PIN         (22u)

// Define ganged axis or A axis step pulse and step direction output pins.
#if N_ABC_MOTORS > 0
#define M3_AVAILABLE
#define M3_STEP_PIN         (8u)
#define M3_DIRECTION_PIN    (9u)
#define M3_LIMIT_PIN        (23u)
#endif

// Define ganged axis or B axis step pulse and step direction output pins.
#if N_ABC_MOTORS == 2
#define M4_AVAILABLE
#define M4_STEP_PIN         (26u)
#define M4_DIRECTION_PIN    (27u)
#define M4_LIMIT_PIN        (28u)
#endif

// Define auxiliary output pins
#define AUXOUTPUT0_PIN      (13u) // Spindle PWM
#define AUXOUTPUT1_PIN      (11u) // Spindle direction
#define AUXOUTPUT2_PIN      (12u) // Spindle enable
#define AUXOUTPUT3_PIN      (19u) // Coolant flood
#define AUXOUTPUT4_PIN      (18u) // Coolant mist

// Define driver spindle pins
#if DRIVER_SPINDLE_ENABLE & SPINDLE_ENA
#define SPINDLE_ENABLE_PIN      AUXOUTPUT2_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_PWM
#define SPINDLE_PWM_PIN         AUXOUTPUT0_PIN
#endif
#if DRIVER_SPINDLE_ENABLE & SPINDLE_DIR
#define SPINDLE_DIRECTION_PIN   AUXOUTPUT1_PIN
#endif

// Define flood and mist coolant enable output pins.
#if COOLANT_ENABLE & COOLANT_FLOOD
#define COOLANT_FLOOD_PIN       AUXOUTPUT3_PIN
#endif
#if COOLANT_ENABLE & COOLANT_MIST
#define COOLANT_MIST_PIN        AUXOUTPUT4_PIN
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold, door) input pins.
#define RESET_PIN           (14u)
#define FEED_HOLD_PIN       (16u)
#define CYCLE_START_PIN     (17u)

// Define auxiliary input pins
#if !QEI_ENABLE
#define AUXINPUT0_PIN       (0u)
#define AUXINPUT1_PIN       (3u)
#endif
#define AUXINPUT2_PIN       (1u)
#define AUXINPUT3_PIN       (29u) // Safety door
#define AUXINPUT4_PIN       (15u) // Probe

#if PROBE_ENABLE
#define PROBE_PIN           AUXINPUT4_PIN
#endif

#if SAFETY_DOOR_ENABLE
#define SAFETY_DOOR_PIN     AUXINPUT3_PIN
#elif MOTOR_FAULT_ENABLE
#define MOTOR_FAULT_PIN     AUXINPUT3_PIN
#endif

// Define probe switch input pin.

#if I2C_ENABLE
#define I2C_PORT            4
#define I2C_SCL4            (24u) // Not referenced, for info only
#define I2C_SDA4            (25u) // Not referenced, for info only
#endif

#if QEI_ENABLE
#define QEI_A_PIN           (0)
#define QEI_B_PIN           (3)
#define QEI_SELECT_PIN      AUXINPUT2_PIN
#endif
