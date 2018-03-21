/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "build_config.h"

#include "system.h"
#include "gpio.h"

#include "sound_beeper.h"


#ifdef BEEPER

void (*systemBeepPtr)(bool onoff) = NULL;

static void beepNormal(bool onoff)
{
    if (onoff) {
        digitalLo(BEEP_GPIO, BEEP_PIN);
    } else {
        digitalHi(BEEP_GPIO, BEEP_PIN);
    }
}

static void beepInverted(bool onoff)
{
    if (onoff) {
        digitalHi(BEEP_GPIO, BEEP_PIN);
    } else {
        digitalLo(BEEP_GPIO, BEEP_PIN);
    }
}
#endif

void systemBeep(bool onoff)
{
#ifndef BEEPER
    UNUSED(onoff);
#else
#if !defined(BEEP_PWM)
    systemBeepPtr(onoff);
#else
    if (onoff)
        TIM_CtrlPWMOutputs(BEEP_TIMER, ENABLE);
    else
        TIM_CtrlPWMOutputs(BEEP_TIMER, DISABLE);
#endif
#endif
}

void beeperInit(beeperConfig_t *config)
{
#ifndef BEEPER
    UNUSED(config);
#else
    initBeeperHardware(config);
    if (config->isInverted)
        systemBeepPtr = beepInverted;
    else
        systemBeepPtr = beepNormal;
    BEEP_OFF;
#endif
}
