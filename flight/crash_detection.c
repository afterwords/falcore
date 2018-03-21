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

#include "io/beeper.h"
#include "io/rc_controls.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/crash_detection.h"
#include "flight/pos_estimation.h"
#include "flight/altitudehold.h"
#include "flight/panic_mode.h"

#include "config/runtime_config.h"


crashState_e g_crashState = NOT_CRASHED;

//static crashDetectionSettings_t * crashDetectionSettings;

/* static float estZVel = 0.0f;
static const float alpha = 0.9f;
static float beta = 1.0f - alpha;

static float alphaTimer = 0.95f;

static bool isCrashingErrorRotationRate = false;    // always valid
static bool isCrashingRotationRate = false;    // always valid
static bool isCrashingAngle = false;           // valid only in ANGLE or HORIZON and ACC present
static bool isCrashingAcceleration = false;    // valid only when ACC is present
static bool isDescendingFast = false; */

/*
#define CRASH_DETECTION_TIME_MS             200     // If crash-like condition is detected for more than 100ms - consider that we are crashing

#define ZERO_RATE_THRESHOLD_DELTA_DPS       50      // 50 deg per second is considered zero
#define CRASHING_RATE_THRESHOLD_DPS         1000    // Over 1000 dps rate is considered crashing
#define CRASHING_RATE_ERROR_THRESHOLD_DPS   50      // over 50 deg per second diffrence between command and actual rate
#define CRASHING_ANGLE_ERROR_THRESHOLD_DEG  30      // over 30 deg diffrence between command and actual rate
#define DESCENT_FAST_THRESHOLD_CMS          800     // 8 meter per second
#define DESCENT_FAST_LAST_TIMER_MS          2000    // 2 sec
#define CRASH_ACCEL_EARTH_Z_CMSS            2000    // 2G */

#define D_ACCEL_TO_THRUST_COEFF_LOW         0.00015
#define D_ACCEL_TO_THRUST_COEFF_MID         0.00020
#define D_ACCEL_TO_THRUST_COEFF_HIGH        0.00035
#define THRUST_MIN_LIMIT                    50

#define GROUND_TOUCHDOWN_ACCEL_CMSS         GRAVITY_CMSS
#define GROUND_FALL_VELOCITY_CMS            -150.0f
#define GROUND_THRESHOLD_ALTITUDE_CM        10

/* Values for blackbox logging */
int16_t g_logCrashDetectLevel;

void switchCrashState(crashState_e newState)
{
    g_crashState = newState;
    
#ifdef DEBUG_CRASH_COUNTER
    switch(g_crashState) {
        case GROUND_IMPACT_LIGHT:
            debug[0]++;
            break;
        case GROUND_IMPACT_MID:
            debug[1]++;
            break;
        case GROUND_IMPACT_HARD:
            debug[2]++;
            break;
        default:
            break;
    }
#endif
}
/* 
void configureCrashDetection(crashDetectionSettings_t * initialCrashDetectionSettings)
{
    crashDetectionSettings = initialCrashDetectionSettings;
} */


void updateGroundImpact(uint32_t currentTime)
{
    static uint32_t previousTime = 0;
	static float  AccelZ_prev = 0.f;
	float thrustAboveMin = 0.f;
	float workThrust = 0.f;
	float dAccelZ = 0.f;
    
	/* delta time */
	uint32_t dT = (currentTime - previousTime);
	previousTime = currentTime;
	
	dAccelZ = imuAccelInEarthFrame.V.Z - AccelZ_prev;
	AccelZ_prev = imuAccelInEarthFrame.V.Z ;
	
	thrustAboveMin = (float)(rcCommand[THROTTLE]  - 1000);
	workThrust = (float)dT * MAX(thrustAboveMin, THRUST_MIN_LIMIT);
	
	if ( ABS(dAccelZ) > (D_ACCEL_TO_THRUST_COEFF_HIGH * workThrust)) {
	    switchCrashState(GROUND_IMPACT_HARD);
	}
	else if ( ABS(dAccelZ) > (D_ACCEL_TO_THRUST_COEFF_MID * workThrust)) {
	    switchCrashState(GROUND_IMPACT_MID);
	}
	else if ( ABS(dAccelZ) > (D_ACCEL_TO_THRUST_COEFF_LOW * workThrust)) {
	    switchCrashState(GROUND_IMPACT_LIGHT);
	}
	else {
	    switchCrashState(NOT_CRASHED);
	}
	
	/* Log crash detection */
	g_logCrashDetectLevel = 100.0f * g_crashState;

#ifdef DEBUG_PANIC_MODE
debug[2] = (1000.0f * ABS(dAccelZ))/(workThrust/1000.0f);
debug[3] = 100.0f * g_crashState;
#endif
	
#ifdef DEBUG_CRASH_IMPACT
debug[0] = 100.0f * g_crashState;
debug[1] = D_ACCEL_TO_THRUST_COEFF * thrustAboveMin*dT_US;
debug[2] = (D_ACCEL_TO_THRUST_COEFF * dT_US * MAX(thrustAboveMin,THRUST_MIN_LIMIT));
debug[3] = ABS(dAccelZ);
#endif
}

bool crashResetEstimatedAltitude() {
	
	/* SNL crash detection delay patch */
	if (g_snlDisableCrashDetection == true && (g_snlState == SNL_LEVEL || g_snlState == SNL_LAND || g_snlState == SNL_DECELERATE ) ) {
		return false;
	}
    /* Detect possible hard touchdown */
		
    if (g_crashState == GROUND_IMPACT_HARD) {
        return true;
    }

    if ((g_crashState == GROUND_IMPACT_LIGHT || g_crashState == GROUND_IMPACT_MID) && (g_estAltitude.est.quality >= EST_ALT_QUAL_MID)) {
        return true;
    }

    if ((g_estAltitude.imu.correctedAccZ > GROUND_TOUCHDOWN_ACCEL_CMSS) &&              // High acceleration in UP direction
            (g_estAltitude.est.aglVelocity < GROUND_FALL_VELOCITY_CMS) &&                   // High velocity in DOWN direction
            (g_estAltitude.est.aglAltitude < GROUND_THRESHOLD_ALTITUDE_CM) &&               // Estimated altitude low (or negative)
            (g_estAltitude.sonar.sonarConsistency > SONAR_CONSISTENCY_LOW_THRESHOLD)) {
        return true;
    }
	
    return false;
}


/* void setZVel(float velocity)
{    
    estZVel = velocity;
    
    if (estZVel > DESCENT_FAST_THRESHOLD_CMS) {
        isDescendingFast = true;
    }
    else {
        isDescendingFast = false;
    }
} */

/* // Make decision on possible crashing based on gyro rates only
void updateCrashDetectionRates(float rpyRequestedRates[3], float rpyActualRates[3], float rpyRateErrors[3])
{
    int axis;
    int crashingAxisCount = 0;
    
    int crashingAxisErrorCount = 0;

    
   // for (axis = 0; axis < 3; axis++) {
   //     if (ABS(rpyRequestedRates[axis]) < ZERO_RATE_THRESHOLD_DELTA_DPS && ABS(rpyActualRates[axis]) > CRASHING_RATE_THRESHOLD_DPS) {
   //         crashingAxisCount++;
   //     }
   // }
    


    for (axis = 0; axis < 3; axis++) {
        if (ABS(rpyRateErrors[axis]) > CRASHING_RATE_ERROR_THRESHOLD_DPS) {
            crashingAxisErrorCount++;
        }
    }
    
    if (crashingAxisErrorCount > 0)
    isCrashingErrorRotationRate = true;
    else 
    isCrashingErrorRotationRate = false;
    
    if (crashingAxisCount > 0)
    isCrashingRotationRate = true;
    else 
    isCrashingRotationRate = false;
} */

/* void updateCrashDetectionAngles(float rpAngleError[2])
{
    static float angleError[2] = {0.0f};
    int axis;
    int crashingAxisCount = 0;
    
    for (axis = 0; axis < 2; axis++) {
        
        angleError[axis] = alpha*angleError[axis] + beta*rpAngleError[axis];
        
        if (ABS(angleError[axis]) > CRASHING_ANGLE_ERROR_THRESHOLD_DEG) {
            crashingAxisCount++;
        }
    }
    
    if (crashingAxisCount > 0)
    isCrashingAngle = true;
    else 
    isCrashingAngle = false;

#ifdef DEBUG_CRASH_DETECTION_ANGLE
    debug[0] = angleError[0];
    debug[1] = angleError[1];
    debug[2] = imuAccelInEarthFrame.V.Z;
    debug[3] = isCrashingAngle;
#endif
}
 */
/* 
void updateCrashDetectionAcceleration(uint32_t currentTime)
{
    static uint32_t descendingFastLastTime = 0;   
    
    
    if (imuAccelInEarthFrame.V.Z > CRASH_ACCEL_EARTH_Z_CMSS && (currentTime - descendingFastLastTime) < MS2US(DESCENT_FAST_LAST_TIMER_MS) ) {
        isCrashingAcceleration = true;
    }else{
        isCrashingAcceleration = false;
    }
    
    if (isDescendingFast)
    descendingFastLastTime = currentTime;

} */
/* 
void updateCrashDetection(uint32_t currentTime)
{
    static uint32_t crashTimer;
    
    static float f_isCrashingRotationRate = 0.0f;
    static float f_isCrashingErrorRotationRate = 0.0f;
    static float f_isCrashingAcceleration = 0.0f;
    static float f_isCrashingAngle = 0.0f;

	// debug ground impact
	updateGroundImpact(currentTime);
	
    // Don't do crash detection if not requested by RC link 
    if (!IS_RC_MODE_ACTIVE(BOXCRASH))
        return;
    
    if (crashDetectionSettings->mode == CRASH_DETECT_ALL_MODES) {
        // Crash detection mode of operation?
    }


    if (!isImuReady()) {
        isCrashingAngle = false;
        isCrashingAcceleration = false;
    }
    else {
        isCrashingAngle = isCrashingAngle && (FLIGHT_MODE(ANGLE_MODE));
    }
    
    f_isCrashingRotationRate =  alphaTimer*f_isCrashingRotationRate + (1.0f-alphaTimer)*(float)isCrashingRotationRate;
    f_isCrashingErrorRotationRate =  alphaTimer*f_isCrashingErrorRotationRate + (1.0f-alphaTimer)*(float)isCrashingErrorRotationRate;
    f_isCrashingAcceleration =  alphaTimer*f_isCrashingAcceleration + (1.0f-alphaTimer)*(float)isCrashingAcceleration;
    f_isCrashingAngle =  alphaTimer*f_isCrashingAngle + (1.0f-alphaTimer)*(float)isCrashingAngle;
    
#ifdef DEBUG_CRASH_DETECTION
    debug[0] = (float)isCrashingErrorRotationRate*100.0f;
    debug[1] = (float)f_isCrashingErrorRotationRate*100.0f;
    debug[2] = (float)isCrashingAngle*100.0f;
    debug[3] = (float)f_isCrashingAngle*100.0f;
#endif
    
    bool possibleCrashDetected = isCrashingErrorRotationRate || isCrashingAngle || isCrashingRotationRate;

    
    if (possibleCrashDetected && !STATE(MOTOR_STOP)) {
        if ((currentTime - crashTimer) >= MS2US(CRASH_DETECTION_TIME_MS)) {
          //  mwDisarm();
        }
    }
    else{
        crashTimer = currentTime;
    }
    
    if (isCrashingAcceleration ) {
       //mwDisarm();
    }
} 
*/
