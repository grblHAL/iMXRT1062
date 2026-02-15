/*
  ioports_analog.c - driver code for iMRXRT1062 ARM processors

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io and PJRC

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

#include "pwm.h"
#include "driver.h"

#include "core_pins.h"

#include "grbl/ioports.h"

typedef struct {
    const pwm_signal_t *pwm;
    ioports_pwm_t pwm_data;
    float pwm_value;
} pwm_ch_t;

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;
static pwm_ch_t *pwm_channels;

// Code lifted from PJRC, pwm.c

FLASHMEM static void set_pwm_cap (xbar_t *output, bool servo_pwm)
{
    if(output && output->id < analog.out.n_ports) {
        aux_out_analog[output->id].mode.pwm = !servo_pwm;
        aux_out_analog[output->id].mode.servo_pwm = servo_pwm;
    }
}

FLASHMEM static uint_fast16_t set_pwm_channels (pwm_config_t *config, ioports_pwm_t *pwm_data)
{
    bool ok;
    uint_fast16_t prescaler = 2, divider = 0b1001;

    if((ok = ioports_precompute_pwm_values(config, pwm_data, F_BUS_ACTUAL / prescaler)))
      while(pwm_data->period > 65534 && divider < 15) {
        prescaler <<= 1;
        divider++;
        ioports_precompute_pwm_values(config, pwm_data, F_BUS_ACTUAL / prescaler);
    }

    return ok ? divider : 0;
}

FLASHMEM static bool init_pwm (xbar_t *output, pwm_config_t *config, bool persistent)
{
    uint32_t prescaler;

    pwm_ch_t *ch = (pwm_ch_t *)output->port;

    if((prescaler = set_pwm_channels(config, &ch->pwm_data)) && pwm_config(ch->pwm, prescaler, ch->pwm_data.period, config->invert))
        set_pwm_cap(output, config->servo_mode);

    return prescaler != 0; 
}

static float pwm_get_value (xbar_t *output)
{
    return pwm_channels && output->id < analog.out.n_ports ? pwm_channels[output->id].pwm_value : -1.0f;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {

        pwm_ch_t ch = pwm_channels[aux_out_analog[port].pwm_idx];

        pwm_out(ch.pwm, ioports_compute_pwm_value(&ch.pwm_data, value));
    }

    return port < analog.out.n_ports;
}

static int32_t wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    return value;
}

FLASHMEM static bool set_function (xbar_t *port, pin_function_t function)
{
    if(port->mode.input)
        aux_out_analog[port->id].id = function;

    return port->mode.input;
}

FLASHMEM static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = NULL;

    memset(&pin, 0, sizeof(xbar_t));

    switch(dir) {

        case Port_Output:
            if(port < analog.out.n_ports) {
                pin.id = port;
                pin.mode = aux_out_analog[pin.id].mode;
                pin.mode.pwm = !pin.mode.servo_pwm; //?? for easy filtering
                XBAR_SET_CAP(pin.cap, pin.mode);
                pin.function = aux_out_analog[pin.id].id;
                pin.group = aux_out_analog[pin.id].group;
                pin.pin = aux_out_analog[pin.id].pin;
                pin.description = aux_out_analog[pin.id].description;
                pin.set_function = set_function;
                if(pin.mode.pwm || pin.mode.servo_pwm) {
                    pin.port = &pwm_channels[aux_out_analog[pin.id].pwm_idx];
                    pin.config = init_pwm;
                    pin.get_value = pwm_get_value;
                }
                info = &pin;
            }
            break;

        default: break;
    }

    return info;
}

FLASHMEM static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
{
    if(dir == Port_Input && port < analog.in.n_ports)
        aux_in_analog[port].description = description;
    else if(port < analog.out.n_ports)
        aux_out_analog[port].description = description;
}

FLASHMEM void ioports_init_analog (pin_group_pins_t *aux_inputs, pin_group_pins_t *aux_outputs)
{
    io_analog_t ports = {
        .ports = &analog,
        .analog_out = analog_out,
        .get_pin_info = get_pin_info,
        .wait_on_input = wait_on_input,
        .set_pin_description = set_pin_description
    };

    aux_in_analog = aux_inputs->pins.inputs;
    aux_out_analog = aux_outputs->pins.outputs;

    analog.in.n_ports = aux_inputs->n_pins;
    analog.out.n_ports = aux_outputs->n_pins;

    if(ioports_add_analog(&ports)) {

        if(analog.out.n_ports) {

            pwm_config_t config = {
                .freq_hz = 5000.0f,
                .min = 0.0f,
                .max = 100.0f,
                .off_value = 0.0f,
                .min_value = 0.0f,
                .max_value = 100.0f,
                .invert = Off
            };

            uint_fast8_t i, n_pwm = 0;
            const pwm_signal_t *pwm;

            for(i = 0; i < analog.out.n_ports; i++) {
                if(aux_out_analog[i].mode.pwm)
                    n_pwm++;
            }

            pwm_channels = calloc(n_pwm, sizeof(pwm_ch_t));

            n_pwm = 0;
            for(i = 0; i < analog.out.n_ports; i++) {
                if(aux_out_analog[i].mode.pwm && !!pwm_channels && (pwm = pwm_claim(NULL, aux_out_analog[i].pin))) {
                    pwm_channels[n_pwm].pwm = pwm;
                    aux_out_analog[i].pwm_idx = n_pwm++;
                    init_pwm(get_pin_info(Port_Output, i), &config, false);
                }
                analog_out(i, 0.0f);
            }
        }
    }
}
