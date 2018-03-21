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

#pragma once

#include "io/escservo.h"
#include "io/rc_controls.h"

#include "common/filter.h"

#include "flight/mixer.h"
#include "flight/pid.h"

typedef enum {
    SNL_INACTIVE = 0,
    SNL_INITIALIZE,
    SNL_DECELERATE,
    SNL_LEVEL,
    SNL_LAND,
    SNL_DISARM,
} snlState_e;

typedef enum {
    FLM_NONE = 0,
    FLM_SHIELD,
    FLM_REGULAR, /* ie ANGLE / HORIZON / ACRO */
} engageSnlFlightMode_e;

typedef struct {
    filterStatePt1_t    targetRollLPFState;
    filterStatePt1_t    targetPitchLPFState;
} filterDecelerationSnl_t;

extern int16_t g_logSnlState;
extern snlState_e g_snlState;
extern engageSnlFlightMode_e g_engageSnlFlightMode;

void initSnlState(void);
void applySnlMode(pidProfile_t * pidProfile, airplaneConfig_t *airplaneConfig, uint32_t currentTime);

