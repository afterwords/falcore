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

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "flight/pid.h"

#pragma once

typedef struct {
    int16_t droneMass;
    uint8_t extraRoll;
    int16_t stickExtraRoll;
    int16_t dirModeValidVel;
    float   yawToRollRatio;
    float   yawStickFactor;
} directionModeSettings_t;

typedef struct {
    int16_t logDirModeVel;
    int16_t logSnlFwdVel;
	int16_t logSnlRightVel;
    int16_t targetRollAngle;
    int16_t targetYawRate;
} g_directionModeLogData_t;

extern float g_estForwardVel; 					 /* will be used in Panic       */
extern float g_estVelBody[2];    

extern g_directionModeLogData_t g_directionModeLogData;

void updateDirectionModeState(void);
void update1AxisVelocityEstimate(void);
void update2AxisVelocityEstimate(void);
void applyDirectionModeController(controlRateConfig_t * controlRateConfig);
void configureDirectionMode(directionModeSettings_t * initialDirectionModeConfig, rxConfig_t * initialRxConfig, pidProfile_t * initialPidProfile);
void reset1AxisVelocityEstimate(void);
void reset2AxisVelocityEstimate(void);
uint8_t isDroneHovering(void);

