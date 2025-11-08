/*

  pwm.c - driver code for iMXRT1062 ARM processor

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

#include "pwm.h"
#include "driver.h"

typedef union {
    IMXRT_FLEXPWM_t *flexpwm;
    IMXRT_TMR_t *qtimer;
} pwm_periph_t;

// Code lifted from PJRC and modified, pwm.c

struct pwm_pin_info_struct {
    uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
    uint8_t module;  // 0-3, 0-3
    uint8_t channel; // 0=X, 1=B, 2=A
    uint8_t muxval;  //
    pwm_periph_t p;
};

#define M(a, b) ((((a) - 1) << 4) | (b))

PROGMEM const struct pwm_pin_info_struct pwm_pin_infos[] = {
    {1, M(1, 1), 0, 4, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_1_X   0  // AD_B0_03
    {1, M(1, 0), 0, 4, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_0_X   1  // AD_B0_02
    {1, M(4, 2), 2, 1, { .flexpwm = &IMXRT_FLEXPWM4 } },    // FlexPWM4_2_A   2  // EMC_04
    {1, M(4, 2), 1, 1, { .flexpwm = &IMXRT_FLEXPWM4 } },    // FlexPWM4_2_B   3  // EMC_05
    {1, M(2, 0), 2, 1, { .flexpwm = &IMXRT_FLEXPWM2 } },    // FlexPWM2_0_A   4  // EMC_06
    {1, M(2, 1), 2, 1, { .flexpwm = &IMXRT_FLEXPWM2 } },    // FlexPWM2_1_A   5  // EMC_08
    {1, M(2, 2), 2, 2, { .flexpwm = &IMXRT_FLEXPWM2 } },    // FlexPWM2_2_A   6  // B0_10
    {1, M(1, 3), 1, 6, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_3_B   7  // B1_01
    {1, M(1, 3), 2, 6, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_3_A   8  // B1_00
    {1, M(2, 2), 1, 2, { .flexpwm = &IMXRT_FLEXPWM2 } },    // FlexPWM2_2_B   9  // B0_11
    {2, M(1, 0), 0, 1, { .qtimer = &IMXRT_TMR1 } },         // QuadTimer1_0  10  // B0_00
    {2, M(1, 2), 0, 1, { .qtimer = &IMXRT_TMR1 } },         // QuadTimer1_2  11  // B0_02
    {2, M(1, 1), 0, 1, { .qtimer = &IMXRT_TMR1 } },         // QuadTimer1_1  12  // B0_01
    {2, M(2, 0), 0, 1, { .qtimer = &IMXRT_TMR2 } },         // QuadTimer2_0  13  // B0_03
    {2, M(3, 2), 0, 1, { .qtimer = &IMXRT_TMR3 } },         // QuadTimer3_2  14  // AD_B1_02
    {2, M(3, 3), 0, 1, { .qtimer = &IMXRT_TMR3 } },         // QuadTimer3_3  15  // AD_B1_03
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {2, M(3, 1), 0, 1, { .qtimer = &IMXRT_TMR3 } },         // QuadTimer3_1  18  // AD_B1_01
    {2, M(3, 0), 0, 1, { .qtimer = &IMXRT_TMR3 } },         // QuadTimer3_0  19  // AD_B1_00
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(4, 0), 2, 1, { .flexpwm = &IMXRT_FLEXPWM4 } },    // FlexPWM4_0_A  22  // AD_B1_08
    {1, M(4, 1), 2, 1, { .flexpwm = &IMXRT_FLEXPWM4 } },    // FlexPWM4_1_A  23  // AD_B1_09
    {1, M(1, 2), 0, 4, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_2_X  24  // AD_B0_12
    {1, M(1, 3), 0, 4, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_3_X  25  // AD_B0_13
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(3, 1), 1, 1, { .flexpwm = &IMXRT_FLEXPWM3 } },    // FlexPWM3_1_B  28  // EMC_32
    {1, M(3, 1), 2, 1, { .flexpwm = &IMXRT_FLEXPWM3 } },    // FlexPWM3_1_A  29  // EMC_31
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(2, 0), 1, 1, { .flexpwm = &IMXRT_FLEXPWM2 } },    // FlexPWM2_0_B  33  // EMC_07
#ifdef ARDUINO_TEENSY40
    {1, M(1, 1), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_1_B  34  // SD_B0_03
    {1, M(1, 1), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_1_A  35  // SD_B0_02
    {1, M(1, 0), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_0_B  36  // SD_B0_01
    {1, M(1, 0), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_0_A  37  // SD_B0_00
    {1, M(1, 2), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_2_B  38  // SD_B0_05
    {1, M(1, 2), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_2_A  39  // SD_B0_04
#endif
#ifdef ARDUINO_TEENSY41
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(2, 3), 2, 6, { .flexpwm = &IMXRT_FLEXPWM2 } },    // FlexPWM2_3_A  36  // B1_00
    {1, M(2, 3), 1, 6, { .flexpwm = &IMXRT_FLEXPWM2 } },    // FlexPWM2_3_B  37  // B1_01
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(1, 1), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_1_B  42  // SD_B0_03
    {1, M(1, 1), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_1_A  43  // SD_B0_02
    {1, M(1, 0), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_0_B  44  // SD_B0_01
    {1, M(1, 0), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_0_A  45  // SD_B0_00
    {1, M(1, 2), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_2_B  46  // SD_B0_05
    {1, M(1, 2), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_2_A  47  // SD_B0_04
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_0_B
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_A
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_B
    {1, M(3, 3), 1, 1, { .flexpwm = &IMXRT_FLEXPWM3 } },    // FlexPWM3_3_B  51  // EMC_22
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_B
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_A
    {1, M(3, 0), 2, 1, { .flexpwm = &IMXRT_FLEXPWM3 } },    // FlexPWM3_0_A  54  // EMC_29
#endif
#ifdef ARDUINO_TEENSY_MICROMOD
    {1, M(1, 1), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_1_B  34  // SD_B0_03
    {1, M(1, 1), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_1_A  35  // SD_B0_02
    {1, M(1, 0), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_0_B  36  // SD_B0_01
    {1, M(1, 0), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_0_A  37  // SD_B0_00
    {1, M(1, 2), 2, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_2_A  38  // SD_B0_04
    {1, M(1, 2), 1, 1, { .flexpwm = &IMXRT_FLEXPWM1 } },    // FlexPWM1_2_B  39  // SD_B0_05
    {2, M(2, 1), 0, 1, { .qtimer = &IMXRT_TMR2 } },         // QuadTimer2_1  40  // B0_04
    {2, M(2, 2), 0, 1, { .qtimer = &IMXRT_TMR2 } },         // QuadTimer2_2  41  // B0_05
    {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_0
    {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_1
    {0, M(1, 0), 0, 0},  // duplicate QuadTimer3_2
    {2, M(4, 0), 0, 1, { .qtimer = &IMXRT_TMR4 } },         // QuadTimer4_0  45  // B0_09
#endif
};

typedef struct {
    const pwm_signal_t *pwm;
} pwm_claimed_t;

uint_fast8_t n_claimed = 0;
pwm_claimed_t pwm_claimed[5] = {0};

FLASHMEM bool pwm_is_available (void *port, uint8_t pin)
{
    const struct pwm_pin_info_struct *pwm = NULL;

    if(pin < sizeof(pwm_pin_infos) / sizeof(struct pwm_pin_info_struct) && pwm_pin_infos[pin].type > 0) {

        uint_fast8_t i;

        pwm = &pwm_pin_infos[pin];

        if(pwm && (i = n_claimed)) do {
            i--;
            if(pwm == pwm_claimed[i].pwm)
                return false;
        } while(i);
    }

    return pwm != NULL;
}

FLASHMEM const pwm_signal_t *pwm_claim (void *port, uint8_t pin)
{
    const struct pwm_pin_info_struct *pwm = NULL;

    if(pwm_is_available(port, pin))
        pwm_claimed[n_claimed++].pwm = pwm = &pwm_pin_infos[pin];

    return pwm;
}

bool pwm_enable (const pwm_signal_t *pwm)
{
    return true;
}

FLASHMEM bool pwm_config (const pwm_signal_t *pwm, uint32_t prescaler, uint32_t period, bool inverted)
{
    uint32_t submodule = pwm->module & 0b11,
             pin = sizeof(pwm_pin_infos) / sizeof(struct pwm_pin_info_struct);

    do {
        if(pwm == &pwm_pin_infos[--pin])
            break;
    } while(pin);

    if(pwm->type == 1) {

        switch((uint32_t)pwm->p.flexpwm) {

            case (uint32_t)&IMXRT_FLEXPWM1:
                CCM_CCGR4 |= CCM_CCGR4_PWM1(CCM_CCGR_ON);
                break;

            case (uint32_t)&IMXRT_FLEXPWM2:
                CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);
                break;

            case (uint32_t)&IMXRT_FLEXPWM3:
                CCM_CCGR4 |= CCM_CCGR4_PWM3(CCM_CCGR_ON);
                break;

            case (uint32_t)&IMXRT_FLEXPWM4:
                CCM_CCGR4 |= CCM_CCGR4_PWM4(CCM_CCGR_ON);
                break;
        }

        pwm->p.flexpwm->FCTRL0 = FLEXPWM_FCTRL0_FLVL(15); // logic high = fault
        pwm->p.flexpwm->FSTS0 = 0x000F; // clear fault status
        pwm->p.flexpwm->FFILT0 = 0;
        pwm->p.flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(15);
        pwm->p.flexpwm->SM[submodule].CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
        pwm->p.flexpwm->SM[submodule].CTRL = FLEXPWM_SMCTRL_FULL | FLEXPWM_SMCTRL_PRSC(prescaler);
        pwm->p.flexpwm->SM[submodule].OCTRL = inverted ? (FLEXPWM_SMOCTRL_POLX << pwm->channel) : 0;
        pwm->p.flexpwm->SM[submodule].DTCNT0 = 0;
        pwm->p.flexpwm->SM[submodule].INIT = 0;
        pwm->p.flexpwm->SM[submodule].VAL0 = 0;
        pwm->p.flexpwm->SM[submodule].VAL1 = period;
        pwm->p.flexpwm->SM[submodule].VAL2 = 0;
        pwm->p.flexpwm->SM[submodule].VAL3 = 0;
        pwm->p.flexpwm->SM[submodule].VAL4 = 0;
        pwm->p.flexpwm->SM[submodule].VAL5 = 0;
        pwm->p.flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(1 << submodule);
        pwm->p.flexpwm->MCTRL |= FLEXPWM_MCTRL_RUN(15);

        *(portConfigRegister(pin)) = pwm->muxval;

    } else if(pwm->type == 2) {

        pwm->p.qtimer->CH[submodule].CTRL = 0;
        pwm->p.qtimer->CH[submodule].SCTRL = inverted ? (TMR_SCTRL_OEN|TMR_SCTRL_FORCE|TMR_SCTRL_VAL|TMR_SCTRL_OPS) : (TMR_SCTRL_OEN|TMR_SCTRL_FORCE|TMR_SCTRL_VAL);
        pwm->p.qtimer->CH[submodule].LOAD = 0;
        pwm->p.qtimer->CH[submodule].COMP1 = period;
        pwm->p.qtimer->CH[submodule].CMPLD1 = period;
        pwm->p.qtimer->CH[submodule].CTRL = TMR_CTRL_CM(0b001) | TMR_CTRL_PCS(prescaler) | TMR_CTRL_LENGTH | TMR_CTRL_OUTMODE(0b100);
        pwm->p.qtimer->ENBL |= 1 << submodule;

        *(portConfigRegister(pin)) = pwm->muxval;
    }

    return pwm->type != 0;
}

bool pwm_out (const pwm_signal_t *pwm, uint32_t pwm_value)
{
    uint32_t submodule = pwm->module & 0b11;

    if(pwm->type == 1) {

        pwm->p.flexpwm->MCTRL |= FLEXPWM_MCTRL_CLDOK(1 << submodule);

        switch(pwm->channel) {

            case 0: // X
                pwm->p.flexpwm->SM[submodule].VAL0 = pwm->p.flexpwm->SM[submodule].VAL1 - pwm_value;
                pwm->p.flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMX_EN(1 << submodule);
                break;

            case 1: // B
                pwm->p.flexpwm->SM[submodule].VAL5 = pwm_value;
                pwm->p.flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMB_EN(1 << submodule);
                break;

            case 2: // A
                pwm->p.flexpwm->SM[submodule].VAL3 = pwm_value;
                pwm->p.flexpwm->OUTEN |= FLEXPWM_OUTEN_PWMA_EN(1 << submodule);
                break;
        }

        pwm->p.flexpwm->MCTRL |= FLEXPWM_MCTRL_LDOK(1 << submodule);

    } else if(pwm->type == 2) {

        pwm->p.qtimer->CH[submodule].COMP2 = pwm_value;
        pwm->p.qtimer->CH[submodule].CMPLD1 = pwm->p.qtimer->CH[submodule].COMP1 - pwm_value;
        pwm->p.qtimer->CH[submodule].CTRL |= TMR_CTRL_CM(0b001);
    }

    return pwm->type != 0;
}

uint32_t pwm_get_clock_hz (const pwm_signal_t *pwm)
{
    return F_BUS_ACTUAL;
}
