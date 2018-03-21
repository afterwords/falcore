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
#include <math.h>

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/gps.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/navigation.h"
#include "flight/gtune.h"
#include "flight/altitudehold.h"
#include "flight/crash_detection.h"
#include "flight/onground_detection.h"

#include "config/runtime_config.h"

extern uint32_t currentTime;
extern uint16_t cycleTime;
extern uint8_t motorCount;
extern float dT;

int16_t axisPID[3];

#ifdef BLACKBOX
int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];
#endif

// PIDweight is a scale factor for PIDs which is derived from the throttle and TPA setting, and 100 = 100% scale means no PID reduction
uint8_t dynP8[3], dynI8[3], dynD8[3], PIDweight[3];

static float errorGyroIf[3] = { 0.0f, 0.0f, 0.0f };

// Dynamic inclination limit calculation
typedef struct {
    bool isUsed;
    float setpoint[2];
    float active[2];
} dynamicAxisInclinationLimit_t;

static dynamicAxisInclinationLimit_t axisAngleLimit[2];

void pidResetCustomInclinationLimits(void)
{
    axisAngleLimit[ROLL].isUsed = false;
    axisAngleLimit[PITCH].isUsed = false;
}

void pidResetCustomAxisInclinationLimits(uint8_t axis)
{
    axisAngleLimit[axis].isUsed = false;
}

void pidSetCustomAxisInclinationLimits(uint8_t axis, int16_t limitPositive, int16_t limitNegative)
{
    axisAngleLimit[axis].setpoint[0] = constrain(limitNegative, 50, 900);
    axisAngleLimit[axis].setpoint[1] = constrain(limitPositive, 50, 900);
}

//#define USE_ANGLE_LIMITING_LPF
#define ANGLE_LIMITING_RATE_DDPS            150     // 15 dps at most
#define ANGLE_LIMITING_CUT_HZ               0.25f
static void calculateActiveInclinationLimit(pidProfile_t *pidProfile)
{
    int axis, direction;

#ifdef USE_ANGLE_LIMITING_LPF
    static filterStatePt1_t angleLimitFilterState[2][2];
#endif

    for (axis = 0; axis < 2; axis++) {
        for (direction = 0; direction < 2; direction++) {
            float axisLimitSetpoint;

            if (axisAngleLimit[axis].isUsed)
                axisLimitSetpoint = constrain(axisAngleLimit[axis].setpoint[direction], 50, pidProfile->max_angle_inclination[axis][direction]);
            else
                axisLimitSetpoint = pidProfile->max_angle_inclination[axis][direction];

#ifdef USE_ANGLE_LIMITING_LPF
            axisAngleLimit[axis].active[direction] = filterApplyPt1(axisLimitSetpoint, &angleLimitFilterState[axis][direction], ANGLE_LIMITING_CUT_HZ, dT);
            axisAngleLimit[axis].active[direction] = constrainf(axisAngleLimit[axis].active[direction], 50, pidProfile->max_angle_inclination[axis][direction]);
#else
            float maxDelta = ANGLE_LIMITING_RATE_DDPS * dT;
            axisAngleLimit[axis].active[direction] += constrainf(axisLimitSetpoint - axisAngleLimit[axis].active[direction], -maxDelta, +maxDelta);
#endif
        }
    }
}

void pidResetErrorGyro(void)
{
    errorGyroIf[ROLL] = 0.0f;
    errorGyroIf[PITCH] = 0.0f;
    errorGyroIf[YAW] = 0.0f;
}

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static filterStatePt1_t deltaFilterState[3];

float rcCommandToAngleDecidegrees(pidProfile_t *pidProfile, uint8_t axis, int32_t rcCommand)
{
    float maxAngle = MAX(pidProfile->max_angle_inclination[axis][0], pidProfile->max_angle_inclination[axis][1]);
    return scaleRange(rcCommand, -500, 500, -maxAngle, maxAngle);
}

int32_t angleDecidegreesToRcCommand(pidProfile_t *pidProfile, uint8_t axis, float angle)
{
    float maxAngle = MAX(pidProfile->max_angle_inclination[axis][0], pidProfile->max_angle_inclination[axis][1]);
    return constrainf(scaleRange(angle, -maxAngle, maxAngle, -500, 500), -500, 500);
}

float rcCommandToAxisRate(uint8_t rate, int32_t rcCommand)
{
    return (float)((rate + 10) * rcCommand) / 50.0f;
}

int32_t axisRateToRcCommand(uint8_t rate, float axisRate)
{
    return axisRate * 50.0f / ((float)rate + 10.0f);     // (float)((rate + 10) * rcCommandYaw) / 50.0f;
}

void pidController(pidProfile_t *pidProfile, controlRateConfig_t *controlRateConfig, rxConfig_t *rxConfig)
{
    static float lastActualRates[3]; //lastRequestedRates[3] //not used

    float RateError, errorAngle, requestedRate, gyroRate;
    float ITerm,PTerm,DTerm;
    int32_t stickPosAil, stickPosEle, mostDeflectedPos;
    static float lastError[3];
    float delta;
    int axis;
    float horizonLevelStrength = 1;
    
    //AMIMON: error angles
    //float errorAngles[2]; //not used

    calculateActiveInclinationLimit(pidProfile);

    if (FLIGHT_MODE(HORIZON_MODE)) {

        // Figure out the raw stick positions
        stickPosAil = getRcStickDeflection(FD_ROLL, rxConfig->midrc);
        stickPosEle = getRcStickDeflection(FD_PITCH, rxConfig->midrc);

        if(ABS(stickPosAil) > ABS(stickPosEle)){
            mostDeflectedPos = ABS(stickPosAil);
        }
        else {
            mostDeflectedPos = ABS(stickPosEle);
        }

        // Progressively turn off the horizon self level strength as the stick is banged over
        horizonLevelStrength = (float)(500 - mostDeflectedPos) / 500;  // 1 at centre stick, 0 = max stick deflection
        if(pidProfile->D8[PIDLEVEL] == 0){
            horizonLevelStrength = 0;
        } else {
            horizonLevelStrength = constrainf(((horizonLevelStrength - 1) * (100 / pidProfile->D8[PIDLEVEL])) + 1, 0, 1);
        }
    }

    // ----------PID controller----------
    for (axis = 0; axis < 3; axis++) {
        // -----Get the desired angle rate depending on flight mode
        uint8_t rate = controlRateConfig->rates[axis];

        if (axis == FD_YAW) {
            requestedRate = rcCommandToAxisRate(rate, rcCommand[YAW]);
         } else {
            // calculate error and limit the angle to the max inclination
#ifdef GPS
            errorAngle = (constrain(rcCommandToAngleDecidegrees(pidProfile, axis, rcCommand[axis]) + GPS_angle[axis],
                                    -axisAngleLimit[axis].active[0],
                                    +axisAngleLimit[axis].active[1]) - attitude.raw[axis]) / 10.0f;
#else
            errorAngle = (constrain(rcCommandToAngleDecidegrees(pidProfile, axis, rcCommand[axis]),
                                    -axisAngleLimit[axis].active[0],
                                    +axisAngleLimit[axis].active[1]) - attitude.raw[axis]) / 10.0f;
#endif
            if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(SNL_MODE)) {
                /*error detection*/
                //errorAngles[axis] = errorAngle;   //not used
                // it's the ANGLE mode - control is angle based, so control loop is needed
                requestedRate = errorAngle * (pidProfile->P8[PIDLEVEL] / PID_LEVEL_P_MULTIPLIER);
            } else {
                //control is GYRO based (ACRO and HORIZON - direct sticks control is applied to rate PID
                requestedRate = rcCommandToAxisRate(rate, rcCommand[axis]);

                if (FLIGHT_MODE(HORIZON_MODE)) {
                    // mix up angle error to desired requestedRate to add a little auto-level feel
                    requestedRate += errorAngle * (pidProfile->I8[PIDLEVEL] / PID_LEVEL_P_MULTIPLIER) * horizonLevelStrength;
                }
            }
        }

        /* TAKEOFF stabilization in IDLE */
        if (isTakeoffAndIdle() && FLIGHT_MODE(ANGLE_MODE)) {
        	requestedRate = 0.0f;
        }

        gyroRate = gyroADC[axis] * gyro.scale; // gyro output scaled to dps

        // --------low-level gyro-based PID. ----------
        // Used in stand-alone mode for ACRO, controlled by higher level regulators in other modes
        RateError = requestedRate - gyroRate;

        // Update state for crash detection
        //lastRequestedRates[axis] = requestedRate; //not used
        lastActualRates[axis] = gyroRate;

        // -----calculate P component
        if (axis == FD_YAW) {
            PTerm = RateError * (pidProfile->P8[axis] / PID_RATE_P_MULTIPLIER) * PIDweight[axis] / 100;
            PTerm = constrain(PTerm, -controlRateConfig->yaw_p_limit, controlRateConfig->yaw_p_limit);
        }
        else {
            PTerm = RateError * (pidProfile->P8[axis] / PID_RATE_P_MULTIPLIER) * PIDweight[axis] / 100;
        }

        // -----calculate I component.
        const float integratorThreshold = (axis == FD_YAW) ? pidProfile->yawItermIgnoreRate : pidProfile->rollPitchItermIgnoreRate;
        const float antiWindupScaler = constrainf(1.0f - (ABS(requestedRate) / integratorThreshold), 0.0f, 1.0f);
        errorGyroIf[axis] = constrainf(errorGyroIf[axis] + RateError * dT * (pidProfile->I8[axis] / PID_RATE_I_MULTIPLIER) * antiWindupScaler, -250.0f, 250.0f);
        ITerm = errorGyroIf[axis];

        //-----calculate D-term
        delta = RateError - lastError[axis];
        lastError[axis] = RateError;

        // Correct difference by cycle time. Cycle time is jittery (can be different 2 times), so calculated difference
        // would be scaled by different dt each time. Division by dT fixes that.
        delta *= (1.0f / dT);

        if (pidProfile->dterm_lpf_hz) {
            delta = filterApplyPt1(delta, &deltaFilterState[axis], pidProfile->dterm_lpf_hz, dT);
        }

        DTerm = constrainf(delta * (pidProfile->D8[axis] / PID_RATE_D_MULTIPLIER) * PIDweight[axis] / 100, -300.0f, 300.0f);

        // -----calculate total PID output
        axisPID[axis] = constrain(lrintf(PTerm + ITerm + DTerm), -1000, 1000);

#ifdef GTUNE
        if (FLIGHT_MODE(GTUNE_MODE) && ARMING_FLAG(ARMED)) {
            calculate_Gtune(axis);
        }
#endif

#ifdef BLACKBOX
        axisPID_P[axis] = PTerm;
        axisPID_I[axis] = ITerm;
        axisPID_D[axis] = DTerm;
#endif
    }

    // Now lastError holds errors between actual rates and requested rates
    //updateCrashDetectionRates(lastRequestedRates, lastActualRates, lastError);
	updateOnGroundDetectionRates(lastActualRates,currentTime);
	updateOnGroundRatesAreStable (lastActualRates,currentTime);
	

   /*  if (FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(SNL_MODE)) {
        updateCrashDetectionAngles(errorAngles);
    } */
}
