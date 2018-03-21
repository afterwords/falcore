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
#include "io/beeper.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/failsafe.h"
#include "flight/panic_mode.h"
#include "flight/direction_mode.h"
#include "flight/altitudehold.h"
#include "flight/pos_estimation.h"
#include "flight/crash_detection.h"

#include "config/config.h"
#include "config/runtime_config.h"

extern altitudeHoldSettins_t * altHoldSettings;

extern uint16_t calcHoverThrottle(uint16_t throttleAtFullBattery);
extern void executeMultirotorSnlAltitudeHold(uint32_t currentTime, float climbRateTargetCMSS, float altitudeTargetCM, bool isRequestingClimbRate);
extern void resetAltHold(void);

#define DISABLE_CRASH_DETECTION_TIMER_MSEC       400
#define DECELERATION_END_TIMER_SEC                 3
#define LEVEL_END_TIMER_SEC                        2
#define SLOW_DESCENT_END_TIMER_SEC                 5.6
#define SNL_END_TIMER_SEC                        100

#define HIGH_ALT_CM                              300 //3 meters
#define FAST_LAND_RATE_CMS                    200.0f
#define SLOW_LAND_RATE_CMS                    100.0f

#define SNL_MIN_VEL_X_CMS                      40.0f
#define SNL_MIN_VEL_Y_CMS                      40.0f
#define MAX_INCLINATION_PITCH_DECIDEG            450
#define MAX_INCLINATION_ROLL_DECIDEG             400
#define DECELERATION_GAIN                          5       // Defines acceleration, unit [1/s]
#define DECELERATION_RATE_LIMIT_DECI_DPS     1500.0f

snlState_e g_snlState = SNL_INACTIVE;
engageSnlFlightMode_e g_engageSnlFlightMode = FLM_NONE;
filterDecelerationSnl_t filterDecelerationSnl;

static bool decelerationCompletedX = false;
static bool decelerationCompletedY = false;
static float snlTargetAltitude = 0.0f;

/*signal pos estimation that we are during the 400MS delay in SNL*/
bool g_snlDisableCrashDetection = false;

/* Values for blackbox logging */
int16_t g_logSnlState;

bool isLeveled(void)
{
	/*  cos(5) =    0.99619469809174553229501040247389 */
	return (calculateCosTiltAngle() >= 0.9961947f); // if we are in a tilt angle greater than 5, returns 0
}

	
void initSnlState(void)
{
    if ((IS_RC_MODE_ACTIVE(BOXSNL) || failsafeIsSNLActive()) && !STATE(FIXED_WING)) {
        if (g_snlState == SNL_INACTIVE) {
            g_snlState = SNL_INITIALIZE;

            if (FLIGHT_MODE(ALTHOLD_DIRECT | ALTHOLD_ASSIST)) {
                g_engageSnlFlightMode = FLM_SHIELD;
            }
            else {
                g_engageSnlFlightMode = FLM_REGULAR;
            }
        }


        ENABLE_FLIGHT_MODE(SNL_MODE);
    }
    else {
#ifdef DEBUG_PANIC_MODE
debug[0]=10;
#endif
		g_snlState = SNL_INACTIVE;
		g_engageSnlFlightMode = FLM_NONE;
		DISABLE_FLIGHT_MODE(SNL_MODE);
    }
}

void resetAngleCommands(void) {
	rcCommand[YAW]   =  0;
	rcCommand[PITCH] =  0;
	rcCommand[ROLL]  =  0;
}

bool applyPanicModeDeceleration(uint32_t currentTime, pidProfile_t * pidProfile)
{
	float targetDecelerationPitchAngle = 0.f;
	float targetDecelerationRollAngle = 0.f;
	static uint32_t previousTime;
	float dT = (currentTime - previousTime) * 1e-6f;
	previousTime = currentTime;

    // Drop velocity
	if ((ABS(g_estVelBody[X]) >= SNL_MIN_VEL_X_CMS) && !decelerationCompletedX) {
		targetDecelerationPitchAngle = -RADIANS_TO_DECIDEGREES(atan2_approx(g_estVelBody[X] * DECELERATION_GAIN, GRAVITY_CMSS));
	}
	else {
		decelerationCompletedX = true;
	}
       
	if ((ABS(g_estVelBody[Y]) >= SNL_MIN_VEL_Y_CMS) && !decelerationCompletedY) {
		targetDecelerationRollAngle = -RADIANS_TO_DECIDEGREES(atan2_approx(g_estVelBody[Y] * DECELERATION_GAIN, GRAVITY_CMSS));
	}
	else {
		decelerationCompletedY = true;
	}
	
	targetDecelerationPitchAngle = filterApplyPt1WithRateLimit(targetDecelerationPitchAngle, &filterDecelerationSnl.targetPitchLPFState, 0, DECELERATION_RATE_LIMIT_DECI_DPS, DECELERATION_RATE_LIMIT_DECI_DPS, dT);
	targetDecelerationRollAngle = filterApplyPt1WithRateLimit(targetDecelerationRollAngle, &filterDecelerationSnl.targetRollLPFState, 0, DECELERATION_RATE_LIMIT_DECI_DPS, DECELERATION_RATE_LIMIT_DECI_DPS, dT);

	rcCommand[PITCH] += angleDecidegreesToRcCommand(pidProfile, PITCH,targetDecelerationPitchAngle);
	rcCommand[ROLL] += angleDecidegreesToRcCommand(pidProfile, ROLL, targetDecelerationRollAngle);

#ifdef DEBUG_PANIC_MODE
debug[1] = targetDecelerationPitchAngle;
//debug[2] = targetDecelerationRollAngle;
#endif

	/* stop deceleration only when both X and Y velocity is under limit */
	if(decelerationCompletedY && decelerationCompletedX) {
		return false;
	}
	else {
		return true;
	}

}

// Callback from the estimator to indicate that AGL was reset
void signalAltitudeResetConditionToSNL(void)
{
    if (FLIGHT_MODE(SNL_MODE) && (g_snlState == SNL_DECELERATE || g_snlState == SNL_LEVEL)) {
        snlTargetAltitude = g_estAltitude.est.aglAltitude;
    }
}

void applySnlMode(pidProfile_t * pidProfile, airplaneConfig_t *airplaneConfig, uint32_t currentTime)
{    
    static uint32_t stopLandStartTime = 0;
    static uint32_t decelerationStartTime = 0;
    static uint32_t levelStartTime = 0;
	static uint32_t slowDescentStartTime = 0;
	 
    beeperMode_e beeperMode = BEEPER_STOP_AND_LAND;

#ifdef DEBUG_PANIC_MODE
//debug 
static uint32_t stopLandDisarmStartTime = 0;
#endif

    if (!ARMING_FLAG(ARMED) ||
        (g_engageSnlFlightMode == FLM_SHIELD && !isTakeoffCompleted())) {
        return;
    }
    
	/* SNL crash detection delay patch */
	if ( (currentTime - stopLandStartTime) > MS2US(DISABLE_CRASH_DETECTION_TIMER_MSEC) ){
		g_snlDisableCrashDetection = false;
	}
	else {
		g_snlDisableCrashDetection = true;
	}
	
    if (g_snlState > SNL_INITIALIZE) {
        resetAngleCommands();
        DISABLE_FLIGHT_MODE(ALTHOLD_ASSIST | ALTHOLD_DIRECT);
        /* stop and land time limiter */
        if ((currentTime - stopLandStartTime) > S2US(SNL_END_TIMER_SEC)) {
            g_snlState = SNL_DISARM;
#ifdef DEBUG_PANIC_MODE
//debug
stopLandDisarmStartTime = currentTime;
debug[0]=33;
#endif
        }
    }

    switch (g_snlState) {
    case SNL_INACTIVE:
        beeperMode = BEEPER_SILENCE;


#ifdef DEBUG_PANIC_MODE
debug[0]=10;
#endif
        g_logSnlState = 0;
        break;

    case SNL_INITIALIZE:

        if (STATE(MOTOR_STOP)) {
            break;
        }

        if (!FLIGHT_MODE(ALTHOLD_DIRECT | ALTHOLD_ASSIST)) {
            g_altHoldHoverThrottle = calcHoverThrottle(altHoldSettings->hover_throttle);
            resetAltHold();           
        }

        stopLandStartTime = currentTime;
        filterResetPt1(&filterDecelerationSnl.targetRollLPFState, attitude.values.roll);
        filterResetPt1(&filterDecelerationSnl.targetPitchLPFState, attitude.values.pitch);

        snlTargetAltitude = g_estAltitude.est.aglAltitude;

		if (ABS(g_estVelBody[X]) <= 8.f*SNL_MIN_VEL_X_CMS && ABS(g_estVelBody[Y]) <= 8.f*SNL_MIN_VEL_Y_CMS  ) {
			g_snlState = SNL_LEVEL;
			levelStartTime = currentTime;
		}
		else {
			g_snlState = SNL_DECELERATE;
			decelerationStartTime = currentTime;
		}
		
		/* reset deceleration conditions */
		decelerationCompletedY = false;
		decelerationCompletedX = false;

#ifdef DEBUG_PANIC_MODE
debug[0]=20;
#endif
        g_logSnlState = 10;
        break;

    case SNL_DECELERATE:
        executeMultirotorSnlAltitudeHold(currentTime, 0.0f, snlTargetAltitude, false);

        if (applyPanicModeDeceleration(currentTime, pidProfile)) {
            // In progress
#ifdef DEBUG_PANIC_MODE
debug[0]=30;
#endif
        }
        else {
            g_snlState = SNL_LEVEL;
            levelStartTime = currentTime;
#ifdef DEBUG_PANIC_MODE
debug[0]=31;
#endif
        }

        if ((currentTime - decelerationStartTime) > S2US(DECELERATION_END_TIMER_SEC)){
            g_snlState = SNL_LEVEL;
#ifdef DEBUG_PANIC_MODE
debug[0]=32;
#endif
        }
		
		/* if we have hard impact DISARM */
        if (g_crashState == GROUND_IMPACT_HARD && g_snlDisableCrashDetection == false ) {
		 g_snlState = SNL_DISARM;
#ifdef DEBUG_PANIC_MODE		 
//debug
stopLandDisarmStartTime = currentTime;
debug[0]=34 ; 
#endif
        g_logSnlState = 20;
        }
        break;

    case SNL_LEVEL:
        executeMultirotorSnlAltitudeHold(currentTime, 0.0f, snlTargetAltitude, false);


#ifdef DEBUG_PANIC_MODE
debug[0]=40;
#endif
        if (isLeveled() || ((currentTime - levelStartTime) > S2US(LEVEL_END_TIMER_SEC) )) {
            resetLandingDetector(currentTime);
            g_snlState = SNL_LAND;
			slowDescentStartTime = currentTime;
#ifdef DEBUG_PANIC_MODE
debug[0]=41;
#endif
        }

		/* if we have hard impact DISARM */
        if (g_crashState == GROUND_IMPACT_HARD && g_snlDisableCrashDetection == false ) {
		 g_snlState = SNL_DISARM;

#ifdef DEBUG_PANIC_MODE
//debug
stopLandDisarmStartTime = currentTime;
debug[0]=43 ; 
#endif
        g_logSnlState = 30;
        }
        break;

    case SNL_LAND:
  
		if (g_estAltitude.sonar.sonarConsistency < SONAR_CONSISTENCY_LIGHT_THRESHOLD){// we have some sonar measurments
            executeMultirotorSnlAltitudeHold(currentTime, -FAST_LAND_RATE_CMS, 0.0f, true);
#ifdef DEBUG_PANIC_MODE
debug[0]=50;
#endif     
			slowDescentStartTime = currentTime;
			g_logSnlState = 40; //fast descent
        } 
        else {
            executeMultirotorSnlAltitudeHold(currentTime, -SLOW_LAND_RATE_CMS, 0.0f, true);
#ifdef DEBUG_PANIC_MODE
debug[0]=51;
#endif
            g_logSnlState = 50;

			/* disarm timer for slow descent stage */
			if ( (currentTime - slowDescentStartTime) > S2US(SLOW_DESCENT_END_TIMER_SEC) ) {
				g_snlState = SNL_DISARM;			
			}
        }

		/* if we have hard impact, or have light impact with sonar on , or low altitude and mid impact */
        const bool isCrashing = ((g_crashState == GROUND_IMPACT_HARD) || (g_crashState == GROUND_IMPACT_MID && g_estAltitude.est.quality >= EST_ALT_QUAL_MID )) && (g_snlDisableCrashDetection == false);
        if ( isLandingDetected(currentTime) || isCrashing ) {
            g_snlState = SNL_DISARM;
        }
	 
        break;

    case SNL_DISARM:
        beeperConfirmationBeeps(2);
        mwDisarm();
        g_logSnlState = 60;
        break;
    }

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}
