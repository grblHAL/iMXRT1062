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

#include "driver.h"

#include "core_pins.h"

#include "grbl/ioports.h"

static io_ports_data_t analog;
static input_signal_t *aux_in_analog;
static output_signal_t *aux_out_analog;
static ioports_pwm_t *pwm_data;
static float *pwm_values;

// Code lifted from PJRC, pwm.c

struct pwm_pin_info_struct {
    uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
    uint8_t module;  // 0-3, 0-3
    uint8_t channel; // 0=X, 1=A, 2=B
    uint8_t muxval;  //
};

#define M(a, b) ((((a) - 1) << 4) | (b))

PROGMEM const struct pwm_pin_info_struct pwm_pin_infos[] = {
    {1, M(1, 1), 0, 4},  // FlexPWM1_1_X   0  // AD_B0_03
    {1, M(1, 0), 0, 4},  // FlexPWM1_0_X   1  // AD_B0_02
    {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04
    {1, M(4, 2), 2, 1},  // FlexPWM4_2_B   3  // EMC_05
    {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06
    {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08
    {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10
    {1, M(1, 3), 2, 6},  // FlexPWM1_3_B   7  // B1_01
    {1, M(1, 3), 1, 6},  // FlexPWM1_3_A   8  // B1_00
    {1, M(2, 2), 2, 2},  // FlexPWM2_2_B   9  // B0_11
    {2, M(1, 0), 0, 1},  // QuadTimer1_0  10  // B0_00
    {2, M(1, 2), 0, 1},  // QuadTimer1_2  11  // B0_02
    {2, M(1, 1), 0, 1},  // QuadTimer1_1  12  // B0_01
    {2, M(2, 0), 0, 1},  // QuadTimer2_0  13  // B0_03
    {2, M(3, 2), 0, 1},  // QuadTimer3_2  14  // AD_B1_02
    {2, M(3, 3), 0, 1},  // QuadTimer3_3  15  // AD_B1_03
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {2, M(3, 1), 0, 1},  // QuadTimer3_1  18  // AD_B1_01
    {2, M(3, 0), 0, 1},  // QuadTimer3_0  19  // AD_B1_00
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08
    {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09
    {1, M(1, 2), 0, 4},  // FlexPWM1_2_X  24  // AD_B0_12
    {1, M(1, 3), 0, 4},  // FlexPWM1_3_X  25  // AD_B0_13
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(3, 1), 2, 1},  // FlexPWM3_1_B  28  // EMC_32
    {1, M(3, 1), 1, 1},  // FlexPWM3_1_A  29  // EMC_31
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(2, 0), 2, 1},  // FlexPWM2_0_B  33  // EMC_07
#ifdef ARDUINO_TEENSY40
    {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
    {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
    {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
    {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
    {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  38  // SD_B0_05
    {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  39  // SD_B0_04
#endif
#ifdef ARDUINO_TEENSY41
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(2, 3), 1, 6},  // FlexPWM2_3_A  36  // B1_00
    {1, M(2, 3), 2, 6},  // FlexPWM2_3_B  37  // B1_01
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  42  // SD_B0_03
    {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  43  // SD_B0_02
    {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  44  // SD_B0_01
    {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  45  // SD_B0_00
    {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  46  // SD_B0_05
    {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  47  // SD_B0_04
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_0_B
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_A
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_B
    {1, M(3, 3), 2, 1},  // FlexPWM3_3_B  51  // EMC_22
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_B
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_A
    {1, M(3, 0), 1, 1},  // FlexPWM3_0_A  54  // EMC_29
#endif
#ifdef ARDUINO_TEENSY_MICROMOD
    {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
    {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
    {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
    {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
    {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  38  // SD_B0_04
    {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  39  // SD_B0_05
    {2, M(2, 1), 0, 1},  // QuadTimer2_1  40  // B0_04
    {2, M(2, 2), 0, 1},  // QuadTimer2_2  41  // B0_05
    {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_0
    {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_1
    {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_2
    {2, M(4, 0), 0, 1},  // QuadTimer4_0  45  // B0_09
#endif
};

// End code lifted from PJRC

static void set_pwm_cap (xbar_t *output, bool servo_pwm)
{
    if(output && output->id < analog.out.n_ports) {
        aux_out_analog[output->id].mode.pwm = !servo_pwm;
        aux_out_analog[output->id].mode.servo_pwm = servo_pwm;
    }
}

static uint_fast16_t set_pwm_values (pwm_config_t *config, ioports_pwm_t *pwm_data)
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
    const struct pwm_pin_info_struct *hw = pwm_pin_infos + output->pin;

    uint32_t prescaler, module, submodule;
    ioports_pwm_t *pwm_data = (ioports_pwm_t *)output->port;
    
    if((prescaler = set_pwm_values(config, pwm_data))) {

        module = (hw->module >> 4) & 0b11;
        submodule = hw->module & 0b11;

        if(hw->type == 1) {

            IMXRT_FLEXPWM_t *flexpwm;

            switch(module) {

                case 0:
                    flexpwm = &IMXRT_FLEXPWM1;
                    CCM_CCGR4 |= CCM_CCGR4_PWM1(CCM_CCGR_ON);
                    break;

                case 1:
                    flexpwm = &IMXRT_FLEXPWM2;
                    CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);
                    break;

                case 2:
                    flexpwm = &IMXRT_FLEXPWM3;
                    CCM_CCGR4 |= CCM_CCGR4_PWM3(CCM_CCGR_ON);
                    break;

                default:
                    flexpwm = &IMXRT_FLEXPWM4;
                    CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);
            }

            flexpwm->FCTRL0 = FLEXPWM_FCTRL0_FLVL(15); // logic high = fault
            flexpwm->FSTS0 = 0x000F; // clear fault status
            flexpwm->FFILT0 = 0;
            flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(15);
            flexpwm->SM[submodule].CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
            flexpwm->SM[submodule].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescaler);
            flexpwm->SM[submodule].OCTRL = 0;
            flexpwm->SM[submodule].DTCNT0 = 0;
            flexpwm->SM[submodule].INIT = 0;
            flexpwm->SM[submodule].VAL0 = 0;
            flexpwm->SM[submodule].VAL1 = pwm_data->period;
            flexpwm->SM[submodule].VAL2 = 0;
            flexpwm->SM[submodule].VAL3 = 0;
            flexpwm->SM[submodule].VAL4 = 0;
            flexpwm->SM[submodule].VAL5 = 0;
            flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(1 << submodule);
            flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(15);

            *(portConfigRegister(output->pin)) = hw->muxval;

        } else if(hw->type == 2) {

            IMXRT_TMR_t *qtimer;

            switch(module) {

                case 0:
                    qtimer = &IMXRT_TMR1;
                    CCM_CCGR6 |= CCM_CCGR6_QTIMER1(CCM_CCGR_ON);
                    break;

                case 1:
                    qtimer = &IMXRT_TMR2;
                    CCM_CCGR6 |= CCM_CCGR6_QTIMER2(CCM_CCGR_ON);
                    break;

                case 2:
                    qtimer = &IMXRT_TMR3;
                    CCM_CCGR6 |= CCM_CCGR6_QTIMER3(CCM_CCGR_ON);
                    break;

                default:
                    CCM_CCGR6 |= CCM_CCGR6_QTIMER4(CCM_CCGR_ON);
                    qtimer = &IMXRT_TMR4;
            }

            qtimer->CH[submodule].CTRL = 0;
            qtimer->CH[submodule].SCTRL = config->invert ? (TMR_SCTRL_OEN|TMR_SCTRL_FORCE|TMR_SCTRL_VAL|TMR_SCTRL_OPS) : (TMR_SCTRL_OEN|TMR_SCTRL_FORCE|TMR_SCTRL_VAL);
            qtimer->CH[submodule].LOAD = 0;
            qtimer->CH[submodule].COMP1 = pwm_data->period;
            qtimer->CH[submodule].CMPLD1 = pwm_data->period;
            qtimer->CH[submodule].CTRL = TMR_CTRL_CM(0b001) | TMR_CTRL_PCS(prescaler) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(0b100);
            qtimer->ENBL |= 1 << submodule;

            *(portConfigRegister(output->pin)) = hw->muxval;
        }
        set_pwm_cap(output, config->servo_mode);
    }
    
    return prescaler != 0; 
}

static float pwm_get_value (xbar_t *output)
{
    return pwm_values && output->id < analog.out.n_ports ? pwm_values[output->id] : -1.0f;
}

static bool analog_out (uint8_t port, float value)
{
    if(port < analog.out.n_ports) {

        uint_fast16_t pwm = ioports_compute_pwm_value(&pwm_data[aux_out_analog[port].pwm_idx], value);
        uint32_t module, submodule;
        const struct pwm_pin_info_struct *hw = pwm_pin_infos + aux_out_analog[port].pin;

        module = (hw->module >> 4) & 0b11;
        submodule = hw->module & 0b11;

        if(pwm_values)
            pwm_values[aux_out_analog[port].id - Output_Analog_Aux0] = value;

        if(hw->type == 1) {

            IMXRT_FLEXPWM_t *flexpwm;

            switch(module) {

                case 0:
                    flexpwm = &IMXRT_FLEXPWM1;
                    break;

                case 1:
                    flexpwm = &IMXRT_FLEXPWM2;
                    break;

                case 2:
                    flexpwm = &IMXRT_FLEXPWM3;
                    break;

                default:
                    flexpwm = &IMXRT_FLEXPWM4;
            }

            flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(1 << submodule);

            switch(hw->channel) {

                case 0: // X
                   flexpwm->SM[submodule].VAL0 = flexpwm->SM[submodule].VAL1 - pwm;
                   flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(1 << submodule);
                   break;

                case 1: // A
                   flexpwm->SM[submodule].VAL3 = pwm;
                   flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << submodule);
                   break;

                case 2: // B
                   flexpwm->SM[submodule].VAL5 = pwm;
                   flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << submodule);
            }

            flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(1 << submodule);

        } else if(hw->type == 2) {

            IMXRT_TMR_t *qtimer;

            switch(module) {

                case 0:
                    qtimer = &IMXRT_TMR1;
                    break;

                case 1:
                    qtimer = &IMXRT_TMR2;
                    break;

                case 2:
                    qtimer = &IMXRT_TMR3;
                    break;

                default:
                    qtimer = &IMXRT_TMR4;
            }

            qtimer->CH[submodule].COMP2 = pwm;
            qtimer->CH[submodule].CMPLD1 = pwm_data[aux_out_analog[port].pwm_idx].period - pwm;
            qtimer->CH[submodule].CTRL |= TMR_CTRL_CM(0b001);
        }
    }

    return port < analog.out.n_ports;
}

static int32_t wait_on_input (uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    return value;
}

static bool set_function (xbar_t *port, pin_function_t function)
{
    if(port->mode.input)
        aux_out_analog[port->id].id = function;

    return port->mode.input;
}

static xbar_t *get_pin_info (io_port_direction_t dir, uint8_t port)
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
                    pin.port = &pwm_data[aux_out_analog[pin.id].pwm_idx];
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

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *description)
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

            for(i = 0; i < analog.out.n_ports; i++) {
                if(aux_out_analog[i].mode.pwm)
                    n_pwm++;
            }

            pwm_data = calloc(n_pwm, sizeof(ioports_pwm_t));
            pwm_values = calloc(n_pwm, sizeof(float));

            n_pwm = 0;
            for(i = 0; i < analog.out.n_ports; i++) {
                if(aux_out_analog[i].mode.pwm && !!pwm_data) {
                    aux_out_analog[i].pwm_idx = n_pwm++;
                    init_pwm(get_pin_info(Port_Output, i), &config, false);
                }
                analog_out(i, 0.0f);
            }
        }
    }
}
