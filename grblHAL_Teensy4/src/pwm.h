/*

  pwm.h - driver code for iMXRT1062 ARM processor

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#pragma once

#include <stdint.h>
#include <stdbool.h>

struct pwm_pin_info_struct;

typedef struct pwm_pin_info_struct pwm_signal_t;

const pwm_signal_t *pwm_claim (void *port, uint8_t pin);
bool pwm_enable (const pwm_signal_t *pwm);
bool pwm_config (const pwm_signal_t *pwm, uint32_t prescaler, uint32_t period, bool inverted);
bool pwm_out (const pwm_signal_t *pwm, uint32_t pwm_value);
bool pwm_is_available (void *port, uint8_t pin);
uint32_t pwm_get_clock_hz (const pwm_signal_t *pwm);
