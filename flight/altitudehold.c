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
#include <math.h>

#include "platform.h"
#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/sonar_hcsr04.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"
#include "sensors/battery.h"

#include "rx/rx.h"

#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/rc_curves.h"
#include "io/beeper.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/pos_estimation.h"
#include "flight/altitudehold.h"
#include "flight/direction_mode.h"
#include "flight/onground_detection.h"

#include "config/config.h"
#include "config/runtime_config.h"

// AMIMON:
// This file is an almost complete rewrite of altitude hold code

//#define SQUARE_STEP_ALT_TUNING
//#define USE_MEASURED_ACCEL_FOR_PID          /* Use measured acceleration (accZ) for calculating velocity derivative */

#define ALTHOLD_UPDATE_HZ                   50  /* 50Hz */
#define TAKEOFF_THROTTLE_UPDATE_HZ          20  /* 20Hz */

#define THROTTLE_CUTOFF_FREQUENCY_HZ        5
#define THROTTLE_INCREASE_RATE_LIMIT        500    /* 500 pwm/s   (0->100% throttle in 2 sec, 10 pwm units per control cycle) working up to 1000 */
#define THROTTLE_DECREASE_RATE_LIMIT        50000  /* pwm/s        (100->0% throttle in 1/50 sec - one control cycle) */

#define SONAR_MIN_TARGET_ALT_CM             5

#define TARGET_ALT_RANGE_BELOW_CM           300
#define TARGET_ALT_RANGE_ABOVE_CM           500     /* max altitude difference for setpoint controller (cm) */

#define PID_ERROR_LIMIT_CM                  500
#define PID_ERROR_LIMIT_CM_SEC              400     /* hardlimit of velocity error to 300 m/s */
#define PID_THROTTLE_INTEGRAL_LIMIT         100     /* Correction limit for the VEL I (~10% throttle) */
#define PID_MAX_ALLOWED_ACCEL_DOWN_CMSS     500     /* Respect hover throttle and ALTHOLD_PD_OUTPUT_LIMIT */
#define PID_MAX_ALLOWED_ACCEL_UP_CMSS       500
#define PID_INTEGRATOR_IGNORE_VEL_CMS       35      /* Disable VEL I after this target velocity */

#define PID_DTERM_LPF_HZ                    8
#define PID_SETALT_LPF_HZ                   2       /* setAlt LPF cutoff */

#define ALTHOLD_PD_OUTPUT_LIMIT             250     /* Limit for AltHold PID output */

#define ALTHOLD_ALT_P_DENOM     10.0f
#define ALTHOLD_ALT_I_DENOM     100.0f
#define ALTHOLD_ALT_FF_DENOM    100.0f

#define ALTHOLD_VEL_P_DENOM     10.0f
#define ALTHOLD_VEL_I_DENOM     10.0f
#define ALTHOLD_VEL_D_DENOM     100.0f

// FIXME: Do actual math here
#define MAX_CELL_VOLTAGE        4.20f
#define MIN_CELL_VOLTAGE        3.30f

#define MIN_STICK_POSITION      135
#define MAX_STICK_POSITION      1000

//#define ENABLE_GROUND_AVOIDANCE

typedef enum {
	TAKEOFF_IDLE,
	TAKEOFF_DETECT_SONAR_CLIMB,
	TAKEOFF_DETECT_SONAR_DESCENT,
	TAKEOFF_COMPLETED,
	TAKEOFF_MANUALLY_ABORTED,
	TAKEOFF_FAILED,
} takeoffStage_e;

/* Hybrid Altitude-to-Velocity-to-Throttle PID controller (Konstantin) */
typedef struct {
	struct {
		float kP;
		float kI;
		float kFF;
	} alt;

	struct {
		float kP;
		float kI;
		float kD;
	} vel;

	float errVelIntegrator;
	float errAltIntegrator;
	float errVel_prev;
	//float lastReference;

	filterStatePt1_t    targetAltLPFState;
	filterStatePt1_t    targetVelLPFState;
	filterStatePt1_t    dtermLPFState;
} pidController_t;

float targetAglAltitude;

int16_t g_altHoldHoverThrottle;
static int16_t deltaThrottleCommand;
static filterStatePt1_t throttleFilterState;
static int16_t initialDeltaThrottleCommand;

/* External parameters */
static pidProfile_t *pidProfile;
static escAndServoConfig_t *escAndServoConfig;
altitudeHoldSettins_t * altHoldSettings;
static rxConfig_t * rxConfig;

/* Altitude hold PID-regulator */
static pidController_t  altHoldPID;

static takeoffStage_e takeoffStage = TAKEOFF_IDLE;
static takeoffStage_e takeoffStage_prev = TAKEOFF_IDLE;

static uint32_t landingTimer;

/* Values for blackbox logging */
altitudeHoldLogData_t g_altitudeHoldLog;

/* AMIMON:
 *  Implementation of PID with back-calculation I-term anti-windup
 *  Control System Design, Lecture Notes for ME 155A by Karl Johan Astrom (p.228)
 *  http://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf */
#ifdef USE_MEASURED_ACCEL_FOR_PID
static float calcAltHoldPID(float targetAlt, float measAlt, float measVel, float measAccZ, float dt, pidController_t * pid, float outMin, float outMax)
#else
static float calcAltHoldPID(float targetAlt, float measAlt, float measVel, float dt, pidController_t * pid, float outMin, float outMax)
#endif
{
	float newProportional, newDerivative, newFeedForward;
	float targetVel_const;
	float max_rate;

	/* This is ~2Hz cutoff 1-st order filter at 50Hz sampling rate
	 * alpha = dt / (dt + RC); RC = 1 / (2*pi*F_cut)
	 * Using filterApplyPt1() for a more generic implementation (immune to sampling rate change if we do it in future) */
	const float targetAlt_f = filterApplyPt1(targetAlt, &pid->targetAltLPFState, PID_SETALT_LPF_HZ, dt);
	const float errAlt = targetAlt_f - measAlt;

	/* Altitude P-controller */
	const float targetVel = errAlt * pid->alt.kP;

	/* Limit setpoint velocity with rate limiting */
	if (!FLIGHT_MODE(SNL_MODE)) {
	    max_rate =  altHoldSettings->max_roc_rod;
	}
	else {
	    max_rate = altHoldSettings->max_snl_rod;
	}
    float targetVel_rlim = filterApplyPt1WithRateLimit(targetVel, &pid->targetVelLPFState, 0, 
                                                       PID_MAX_ALLOWED_ACCEL_UP_CMSS, PID_MAX_ALLOWED_ACCEL_DOWN_CMSS, dt);
    targetVel_const = constrainf(targetVel_rlim, -max_rate, max_rate);

	/* Feed-forward */
	newFeedForward = errAlt * pid->alt.kFF;

	/* Altitude I-controller */
	/* This integrator compensates for ground effect (different pressure than ambience) when measVel might be slightly incorrect due to
	 * being calculated from baro as well. Incorrect measVel might also be caused by slightly incorrect accelerometer bias as it is also
	 * calculated based on baro). Integrator adjusts the target velocity accordingly to minimise errAlt */
	float errAlt_const = constrainf(errAlt , -(PID_ERROR_LIMIT_CM_SEC)/40.0f, (PID_ERROR_LIMIT_CM_SEC)/40.0f);
	pid->errAltIntegrator  = constrainf(pid->errAltIntegrator + (errAlt_const * pid->alt.kI * dt),
	                                   -max_rate/8.0f, max_rate/8.0f);
	/* Velocity PID-controller starts here */

	/* errVel is altitude PI-controller output */
	float errVel = (targetVel_const + pid->errAltIntegrator) - measVel;
	float errVel_const = constrainf(errVel, -PID_ERROR_LIMIT_CM_SEC, PID_ERROR_LIMIT_CM_SEC);

	/* P-term */
	newProportional = errVel_const * pid->vel.kP;

	/* Measurement tracking D-term */
#ifdef USE_MEASURED_ACCEL_FOR_PID
	newDerivative = pid->vel.kD * (-measAccZ);
#else
	newDerivative = pid->vel.kD * ((errVel_const - pid->errVel_prev) / dt);
	pid->errVel_prev = errVel_const;

#ifdef DEBUG_ACCEL_PID
	debug[0] = 100.0f* pid->vel.kD * (-measAccZ);
	debug[1] = 100.0f* newDerivative;
#endif

#ifdef DEBUG_ACCEL_PID
	debug[3] = 100.0f*newDerivative;
#endif
#endif

	/* Constrain D-term to prevent actuator saturation */
	newDerivative = constrainf(newDerivative, outMin, outMax);

	/* Apply D-term low-pass filter */
	newDerivative = filterApplyPt1(newDerivative, &pid->dtermLPFState, PID_DTERM_LPF_HZ, dt);

	/* limit the integrator error */
    const float antiWindupVelScaler = constrainf(1.0f - (ABS(targetVel_const) / PID_INTEGRATOR_IGNORE_VEL_CMS), 0.1f, 1.0f);    // Allow 10% of kI always
	float newIntegrator = pid->errVelIntegrator + (errVel_const * pid->vel.kI * antiWindupVelScaler * dt);
	pid->errVelIntegrator = constrainf(newIntegrator, MIN(outMin * 0.2f, -PID_THROTTLE_INTEGRAL_LIMIT), MAX(outMax * 0.2f, PID_THROTTLE_INTEGRAL_LIMIT));

	/* Pre-calculate output and limit it if actuator is saturating */
	float outVal = constrainf(newProportional + newDerivative, -ALTHOLD_PD_OUTPUT_LIMIT, ALTHOLD_PD_OUTPUT_LIMIT)
                    + pid->errVelIntegrator;

	/* Apply feed-forward term and limit */
	const float throttleCorrection = constrainf(outVal + newFeedForward, outMin, outMax);

#ifdef DEBUG_ALTITUDE_PID
	debug[0] = pid->errVelIntegrator;
	debug[1] = pid->errAltIntegrator;
	debug[2] = errVel_const;
	debug[3] = errAlt;
#endif

	/* Log */
	g_altitudeHoldLog.logTargetAlt = targetAlt;
	g_altitudeHoldLog.logTargetVel = targetVel_const;
	g_altitudeHoldLog.logPidP = newProportional;
	g_altitudeHoldLog.logPidI = pid->errVelIntegrator;
	g_altitudeHoldLog.logPidD = newDerivative;
	g_altitudeHoldLog.logPidF = pid->errAltIntegrator; //newFeedForward;

	return throttleCorrection;
}

void resetAltHoldPID(pidController_t * pid)
{
	/* Reset all integrators and filters */
	pid->errVelIntegrator = 0.0f;
	pid->errAltIntegrator = 0.0f;
	pid->errVel_prev = 0.0f;

	filterResetPt1(&pid->dtermLPFState, 0);
	filterResetPt1(&pid->targetAltLPFState, 0);
	if (takeoffStage == TAKEOFF_COMPLETED) {
	    filterResetPt1(&pid->targetVelLPFState, g_estAltitude.est.aglVelocity);
	}
	else {
	    filterResetPt1(&pid->targetVelLPFState, 0);
	}
}

void configureAltitudeHoldPID(void)
{
	/* Set AltHold PIDs for Altitude and Velocity loops
	 * Initialize all PIDs in AltHold
	 * Alt PI+FF */
	altHoldPID.alt.kP = pidProfile->P8[PIDALT] / ALTHOLD_ALT_P_DENOM;         /* Alt P-gain */
	altHoldPID.alt.kI = pidProfile->I8[PIDALT] / ALTHOLD_ALT_I_DENOM;         /* Alt I-gain */
	altHoldPID.alt.kFF = pidProfile->D8[PIDALT] / ALTHOLD_ALT_FF_DENOM;       /* Alt Feed-forward gain */

	/* Vel PID */
	altHoldPID.vel.kP = pidProfile->P8[PIDVEL] / ALTHOLD_VEL_P_DENOM;         /* Vel P-gain */
	altHoldPID.vel.kI = pidProfile->I8[PIDVEL] / ALTHOLD_VEL_I_DENOM;         /* Vel I-gain */
	altHoldPID.vel.kD = pidProfile->D8[PIDVEL] / ALTHOLD_VEL_D_DENOM;         /* Vel D-gain */

	resetAltHoldPID(&altHoldPID);
}

bool isThrustFacingDownwards(void)
{
	/* Tilt angle <= 80 deg; cos(80) = 0.17364817766693034885171662676931 */
	return (calculateCosTiltAngle() >= 0.173648178f);
}

#if defined(BARO) || defined(SONAR)

float calcBatteryCellVoltageRatio(void)
{
	if (feature(FEATURE_VBAT)) {
		/* Compensate for partially discharged bat */
		float cellVoltage = (vbatSmooth * 0.1f) / batteryCellCount;   /* vbat is in 0.1V steps */
		return constrainf(cellVoltage, MIN_CELL_VOLTAGE, MAX_CELL_VOLTAGE) / MAX_CELL_VOLTAGE;
	}
	else
	{
		return 1.0f;
	}
}

		
uint16_t calcHoverThrottle(uint16_t throttleAtFullBattery)
{
	if (feature(FEATURE_VBAT)) {
		int32_t compensatedThrottle = escAndServoConfig->minthrottle + (throttleAtFullBattery - escAndServoConfig->minthrottle) / calcBatteryCellVoltageRatio();
		return constrain(compensatedThrottle, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
	}
	else { /* no VBAT sensor */
		return throttleAtFullBattery;
	}
}

void resetAltHold(void)
{
	/* Called when altitude hold PID state may be undefined or outdated */
	resetAltHoldPID(&altHoldPID);
	throttleFilterState.state = 0.0f;
	deltaThrottleCommand = 0;
	initialDeltaThrottleCommand = 0;
}

static void takeoffSwitchStage(takeoffStage_e newStage)
{
	takeoffStage_prev = takeoffStage;
	takeoffStage = newStage;
}

static int getRelativeThrottleStickPosition(int16_t rcThrottle, int deadbandSize)  /* [0..1000] range with deadband at middle */
{
    rcThrottle = constrain(rcThrottle, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);

    if (rcThrottle <= (lookupThrottleRCMid - deadbandSize)) {
        return scaleRange(rcThrottle, escAndServoConfig->minthrottle, lookupThrottleRCMid - deadbandSize, 0, 500);
    }
    else if (rcThrottle >= (lookupThrottleRCMid + deadbandSize)) {
        return scaleRange(rcThrottle, lookupThrottleRCMid + deadbandSize, escAndServoConfig->maxthrottle, 500, 1000);
    }
    else { /* stick within the deadband */
        return 500;
    }
}

void configureAltitudeHold(pidProfile_t *initialPidProfile,
		altitudeHoldSettins_t * initialAltHoldSettings,
		escAndServoConfig_t *initialEscAndServoConfig,
		rxConfig_t * initialRxConfig)
{
	pidProfile = initialPidProfile;
	escAndServoConfig = initialEscAndServoConfig;
	altHoldSettings = initialAltHoldSettings;
	rxConfig = initialRxConfig;

	/* Set hover throttle hint (safeguard in case TAKE_OFF is not used - i.e. BARO-only operation) */
	g_altHoldHoverThrottle = calcHoverThrottle(altHoldSettings->hover_throttle);

	configureAltitudeHoldPID();
    resetEstimatedAltitude();
	resetAltHold();
}

void resetLandingDetector(uint32_t currentTime)
{
    landingTimer = currentTime;
}

static void updateLandingDetector(uint32_t currentTime, int throttleCommand)
{
    // Check if our throttle is below 25% of nominal hover throttle, our RoC/RoD is below 15cm/s and we are not upside down
    const bool verticalMovement = ABS(g_estAltitude.est.aglVelocity) > 15.0f;
    const bool isAtMinimalThrust = (throttleCommand - escAndServoConfig->minthrottle) < ((altHoldSettings->hover_throttle - escAndServoConfig->minthrottle) * 0.25f);
    const bool isPossibleLandingDetected = !verticalMovement && isAtMinimalThrust && isThrustFacingDownwards();

    if (!isPossibleLandingDetected) {
        landingTimer = currentTime;
    }
}

bool isLandingDetected(uint32_t currentTime)
{
    // This function is safe to call only if we are executing altitude hold controller, otherwise it will return TRUE regardless
    // Landing condition satisfied for 1 second, we are landed
    return ((currentTime - landingTimer) > S2US(1));
}

#ifdef USE_MEASURED_ACCEL_FOR_PID
static void calculateThrottleAdjustment(uint32_t deltaTime, float targetAlt, float estAlt, float estVel, float estAccZ)
#else
static void calculateThrottleAdjustment(uint32_t deltaTime, float targetAlt, float estAlt, float estVel)
#endif
{
	float dT = deltaTime * 1e-6;

	int32_t thrAdjustmentMin = escAndServoConfig->minthrottle - g_altHoldHoverThrottle;
	int32_t thrAdjustmentMax = escAndServoConfig->maxthrottle - g_altHoldHoverThrottle;

	if (isThrustFacingDownwards()) { /* we are in the upright position */
#ifdef USE_MEASURED_ACCEL_FOR_PID
	    deltaThrottleCommand = calcAltHoldPID(targetAlt, estAlt, estVel, estAccZ, dT, &altHoldPID, thrAdjustmentMin, thrAdjustmentMax);
#else
        deltaThrottleCommand = calcAltHoldPID(targetAlt, estAlt, estVel, dT, &altHoldPID, thrAdjustmentMin, thrAdjustmentMax);
#endif
	}
	else { /* we are upside-down or about to go upside down - bring throttle to a minimum */
		deltaThrottleCommand = thrAdjustmentMin;
	}

	/* Apply low-pass filter and limit correction to max possible range */
    if (FLIGHT_MODE(ALTHOLD_DIRECT)) {
        deltaThrottleCommand = filterApplyPt1(deltaThrottleCommand, &throttleFilterState, THROTTLE_CUTOFF_FREQUENCY_HZ, dT);
    }
    else {  // ALTHOLD_ASSIST
        deltaThrottleCommand = filterApplyPt1WithRateLimit(deltaThrottleCommand, &throttleFilterState, THROTTLE_CUTOFF_FREQUENCY_HZ,
                                                           THROTTLE_INCREASE_RATE_LIMIT, THROTTLE_DECREASE_RATE_LIMIT, dT);
    }

	deltaThrottleCommand = constrain(deltaThrottleCommand, thrAdjustmentMin, thrAdjustmentMax);

	initialDeltaThrottleCommand = deltaThrottleCommand;
	
	/* Log */
	g_altitudeHoldLog.altHoldAdjustment = deltaThrottleCommand;
}

void groundAvoidance(float measAlt)
{
	float errAlt = targetAglAltitude - measAlt;

	/* matlab: ezsurf('(1-(x)*(1-y))*0.8+.2', [0 1 0 1]) */
	float factor = (1.0 - constrainf(-errAlt/100,0,1) * (1 - constrainf(targetAglAltitude / 100,0,1))) * 0.8 + 0.2;
	if (errAlt < 0) { /* descent command */
		errAlt *= constrainf(factor,0.2,1.0);
		targetAglAltitude = errAlt + measAlt;
	}

#ifdef DEBUG_LIMIT_DESCENT_RATE
	debug[0] = errAlt;
	debug[1] = targetAglAltitude;
	debug[2] = constrainf(100*factor,0, 100);
	debug[3] = errAlt+measAlt;
#endif
}

void executeMultirotorSnlAltitudeHold(uint32_t currentTime, float climbRateTargetCMSS, float altitudeTargetCM, bool isRequestingClimbRate)
{
	static uint32_t previousTime;
	uint32_t deltaTime = currentTime - previousTime;

	if (deltaTime > (2 * HZ2US(ALTHOLD_UPDATE_HZ))) {
		/* Previous correction was too long ago ignore this update and force reset state */
		previousTime = currentTime;
	}
	else if (deltaTime > HZ2US(ALTHOLD_UPDATE_HZ)) {
        float panicTargetAltitude;

		previousTime = currentTime;
            
        if (isRequestingClimbRate) {
            panicTargetAltitude = g_estAltitude.est.aglAltitude + (climbRateTargetCMSS / altHoldPID.alt.kP); /* alt descend control */
        }
        else {
            panicTargetAltitude = altitudeTargetCM;
        }

#ifdef USE_MEASURED_ACCEL_FOR_PID
        calculateThrottleAdjustment(deltaTime, panicTargetAltitude, g_estAltitude.est.aglAltitude, g_estAltitude.est.aglVelocity, g_estAltitude.imu.correctedAccZ);
#else
        calculateThrottleAdjustment(deltaTime, panicTargetAltitude, g_estAltitude.est.aglAltitude, g_estAltitude.est.aglVelocity);
#endif

	}

	/* Apply correction to throttle */
	rcCommand[THROTTLE] = constrain(g_altHoldHoverThrottle + deltaThrottleCommand, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
    updateLandingDetector(currentTime, rcCommand[THROTTLE]);
}

static void executeMultirotorAltHold(throttleStatus_e throttleStatus, uint32_t currentTime)
{
	static uint32_t previousTime;
	uint32_t deltaTime = currentTime - previousTime;
	float throttleSlewRate = 0.0f;
	

	if (!ARMING_FLAG(ARMED)) {
		return;
	}

	if (throttleStatus == THROTTLE_LOW && feature(FEATURE_MOTOR_STOP)) {    /* props are not spinning - avoid windup */
		return;
	}

	if (deltaTime > (2 * HZ2US(ALTHOLD_UPDATE_HZ))) {
		/* Previous correction was too long ago ignore this update and force reset state */
		previousTime = currentTime;
		resetAltHold();
	}
	else if (deltaTime >= HZ2US(ALTHOLD_UPDATE_HZ)) {
		/* Right + on time - update throttle correction */
		previousTime = currentTime;

        if (FLIGHT_MODE(ALTHOLD_DIRECT)) {
			/* command land when throttle is low */
		
			
			int stickPos = getRelativeThrottleStickPosition(rcStickCommand[THROTTLE], 0);

			#ifdef DEBUG_LOW_STICK_THROTTLE
			debug[1] = stickPos;
			#endif
			
			if( stickPos <= MIN_STICK_POSITION && g_estAltitude.est.aglAltitude <= (SONAR_MIN_TARGET_ALT_CM + 5.0f) ) {
		
				throttleSlewRate = constrainf(MIN_LAND_THROTTLE_SLEW_RATE + MAX_LAND_THROTTLE_SLEW_RATE * (  initialDeltaThrottleCommand -  deltaThrottleCommand ) / ( g_altHoldHoverThrottle + initialDeltaThrottleCommand  - escAndServoConfig->minthrottle ) ,MIN_LAND_THROTTLE_SLEW_RATE,MAX_LAND_THROTTLE_SLEW_RATE);

				deltaThrottleCommand = deltaThrottleCommand  - (int16_t)throttleSlewRate;
			
				if( !STATE(MOTOR_STOP) && ( g_altHoldHoverThrottle + deltaThrottleCommand ) <= escAndServoConfig->minthrottle  ) {
				    takeoffSwitchStage(TAKEOFF_IDLE);
					ENABLE_STATE(TAKE_OFF);
				}
				
		
			#ifdef DEBUG_LOW_STICK_THROTTLE
			debug[0] = 100*(int16_t)throttleSlewRate;
			#endif
	
			}			
			else {
				
				#ifdef USE_MEASURED_ACCEL_FOR_PID
				calculateThrottleAdjustment(deltaTime, targetAglAltitude, g_estAltitude.est.aglAltitude, g_estAltitude.est.aglVelocity, g_estAltitude.imu.correctedAccZ);
				#else
				calculateThrottleAdjustment(deltaTime, targetAglAltitude, g_estAltitude.est.aglAltitude, g_estAltitude.est.aglVelocity);
				#endif
		        
				
			}
        }
        else if (FLIGHT_MODE(ALTHOLD_ASSIST)) {
            // FIXME
        }

	}

	/* Apply correction to throttle */
	rcCommand[THROTTLE] = constrain(g_altHoldHoverThrottle + deltaThrottleCommand, escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
    updateLandingDetector(currentTime, rcCommand[THROTTLE]);

	#ifdef DEBUG_LOW_STICK_THROTTLE
	debug[2] = deltaThrottleCommand;
	debug[3] = g_altHoldHoverThrottle + deltaThrottleCommand;
	#endif

}





static void executeMultirotorTakeOff(uint32_t currentTime)
{
	static uint32_t takeoffStartTime = 0;
	static uint32_t takeoffAbortStartTime = 0;
	static float rcThrottleCommand;
	static uint32_t previousTime;
	float throttleSlewRate;
	float takeoffThrottleSlewRate;
	uint32_t takeoffClimbTimeOutUS;
	uint32_t takeoffDescentTimeOutUS;
	uint16_t takeoffPropGuardMargin;
    batteryState_e batState = getBatteryState();

	
	const uint32_t maxTakeoffThrottle = constrain(calcHoverThrottle(altHoldSettings->hover_throttle) + TAKEOFF_THROTTLE_ERROR_MARGIN,
			                                      escAndServoConfig->minthrottle, escAndServoConfig->maxthrottle);
   
	/* calculate battery compensation */
	
	takeoffThrottleSlewRate = constrainf(TAKEOFF_THROTTLE_SLEW_RATE / calcBatteryCellVoltageRatio(), TAKEOFF_THROTTLE_SLEW_RATE,2.0f * TAKEOFF_THROTTLE_SLEW_RATE);
	takeoffPropGuardMargin = constrain(TAKEOFF_PROP_GUARD_MARGIN / calcBatteryCellVoltageRatio(), TAKEOFF_PROP_GUARD_MARGIN, 2.0f * TAKEOFF_PROP_GUARD_MARGIN);
	takeoffClimbTimeOutUS = (uint32_t) S2US(constrainf(TAKEOFF_TIMEOUT_CLIMB_SEC/calcBatteryCellVoltageRatio(), TAKEOFF_TIMEOUT_CLIMB_SEC, 2.0f * TAKEOFF_TIMEOUT_CLIMB_SEC));
	takeoffDescentTimeOutUS	= (uint32_t) S2US(constrainf(TAKEOFF_TIMEOUT_DESCENT_SEC/calcBatteryCellVoltageRatio(), TAKEOFF_TIMEOUT_DESCENT_SEC,2.0f * TAKEOFF_TIMEOUT_DESCENT_SEC));
	
	const uint32_t minHoverThrottle  = maxTakeoffThrottle - takeoffPropGuardMargin;
	
	/* calculate throttle stick threshold */
	const int16_t takeoffStickThreshold = rxConfig->mincheck + 0.5f * (lookupThrottleRCMid - rxConfig->mincheck);

	uint32_t deltaTime = currentTime - previousTime;

	/* Can't do anything until machine is armed */
	if (!ARMING_FLAG(ARMED)) {
		takeoffStage = TAKEOFF_IDLE;
		takeoffStage_prev = TAKEOFF_IDLE;
		return;
	}

		#ifdef DEBUG_HOVER_RATIO
		debug[0] = 100.0f*takeoffThrottleSlewRate;
		debug[1] = 100.0f*takeoffClimbTimeOutUS;
		debug[2] = takeoffStickThreshold;
		#endif
		
	switch (takeoffStage) {
	case TAKEOFF_IDLE:
	
		/* Takeoff pending, but not started yet, reset PID integral to avoid motor spool-up when idle */
		pidResetErrorGyro();

		/* we assume we are on the ground */
		resetEstimatedAltitude();

		/* Keep motors at minimum at this stage */
		rcCommand[THROTTLE] = escAndServoConfig->minthrottle;

        /* Check if its safe to take off */
        if (!isThrustFacingDownwards() ||
            (feature(FEATURE_VBAT) && (batState == BATTERY_CRITICAL || batState == BATTERY_WARNING || batState == BATTERY_NOT_PRESENT))) {
            takeoffSwitchStage(TAKEOFF_FAILED);
            break;
        }

		/* raising the throttle over the 1/4  will make it takeoff */
		if(rcStickCommand[THROTTLE] > (takeoffStickThreshold + TAKEOFF_THROTTLE_HYSTERESIS)){  /* Reset takeoff start time so we can track how long takeoff procedure is taking */
			rcThrottleCommand = escAndServoConfig->minthrottle;
			takeoffStartTime = currentTime;
			previousTime = currentTime;

			takeoffSwitchStage(TAKEOFF_DETECT_SONAR_CLIMB);
		}
		g_altitudeHoldLog.logTakeoffState = 0;
		break;

	case TAKEOFF_DETECT_SONAR_CLIMB: /* Main takeoff logic */

		switchOnGroundDetectionState(UNDETERMINED_ONGROUND);

		if (deltaTime > HZ2US(TAKEOFF_THROTTLE_UPDATE_HZ)) { /* raise throttle at reasonable rate */
			previousTime = currentTime;

			/* increase throttle non linear and linear at the end*/
			if( rcThrottleCommand <= minHoverThrottle ){ 
			throttleSlewRate = 3.0 * takeoffThrottleSlewRate * (maxTakeoffThrottle - rcThrottleCommand) / (maxTakeoffThrottle - escAndServoConfig->minthrottle);
			}
			else {
			throttleSlewRate =  takeoffThrottleSlewRate/3.0; 	
			}
			
#ifdef DEBUG_HOVER_RATIO
debug[3] = throttleSlewRate;
#endif
			rcThrottleCommand =  rcThrottleCommand + throttleSlewRate;

			/* don't go over what we think is enough for hovering */
			rcThrottleCommand = constrainf(rcThrottleCommand, escAndServoConfig->minthrottle, maxTakeoffThrottle);
		}

		/* send output to motors (not only when rcThrottleCommand is updated, but always) */
		rcCommand[THROTTLE] = (int16_t)rcThrottleCommand;

				
		/* pilot cancelled the takeoff taking stick below 25% */
		if (rcStickCommand[THROTTLE] < (takeoffStickThreshold - TAKEOFF_THROTTLE_HYSTERESIS)) {    /* below 1/4 throttle */
			rcThrottleCommand = escAndServoConfig->minthrottle;
			takeoffSwitchStage(TAKEOFF_MANUALLY_ABORTED);
			break;
		}

		if ( ( (currentTime - takeoffStartTime) > takeoffClimbTimeOutUS ) || (g_estAltitude.est.aglAltitude >= (altHoldSettings->sonar_max_alt * 0.5f) ) ) { /* time limit for successful takeoff */
			/* try to detect sonar during descent */
			takeoffSwitchStage(TAKEOFF_DETECT_SONAR_DESCENT);
			/* init the idle down timer */
			takeoffAbortStartTime = currentTime;
			break;
		}

		if (g_estAltitude.est.quality == EST_ALT_QUAL_HIGH && g_estAltitude.est.aglAltitude >= (altHoldSettings->sonar_max_alt * 0.25f)) { /* we detect alt above limit and good sonar quality */
			takeoffSwitchStage(TAKEOFF_COMPLETED);
			break;
		}

		g_altitudeHoldLog.logTakeoffState = 10;
		break;

	case TAKEOFF_DETECT_SONAR_DESCENT:
		switchOnGroundDetectionState(UNDETERMINED_ONGROUND);

		/* raise throttle at reasonable rate */
		if (deltaTime > HZ2US(TAKEOFF_THROTTLE_UPDATE_HZ)) {
			previousTime = currentTime;

			/* close throttle non linearly */
			throttleSlewRate =  -4.0 * takeoffThrottleSlewRate * ( rcThrottleCommand - escAndServoConfig->minthrottle) / (maxTakeoffThrottle - escAndServoConfig->minthrottle);
			
			rcThrottleCommand =  rcThrottleCommand + throttleSlewRate;

			/* don't go over what we think is enough for hovering */
			rcThrottleCommand = constrainf(rcThrottleCommand, escAndServoConfig->minthrottle, maxTakeoffThrottle);
		}

		/* send output to motors (not only when rcThrottleCommand is updated, but always */
		rcCommand[THROTTLE] = (int16_t)rcThrottleCommand;

		/* pilot cancelled the takeoff taking stick below 25% */
		if (rcStickCommand[THROTTLE] < (takeoffStickThreshold - TAKEOFF_THROTTLE_HYSTERESIS)) {    /* below 1/4 throttle */
			rcThrottleCommand = escAndServoConfig->minthrottle;
			takeoffSwitchStage(TAKEOFF_MANUALLY_ABORTED);
			break;
		}

		/* time limit for successful takeoff */
		if ((currentTime - takeoffAbortStartTime) > takeoffDescentTimeOutUS || imuAccelInEarthFrame.V.Z > CRASH_ACCEL_EARTH_Z_CMSS) {
			takeoffSwitchStage(TAKEOFF_FAILED);
			break;
		}

		/* conditions for completed takeoff */
		//if (rcThrottleCommand > minHoverThrottle && g_estAltitude.est.quality == EST_ALT_QUAL_HIGH && g_estAltitude.est.aglAltitude >=  (altHoldSettings->sonar_max_alt * 0.25f)) {
		if (g_estAltitude.est.quality == EST_ALT_QUAL_HIGH && g_estAltitude.est.aglAltitude >=  (altHoldSettings->sonar_max_alt * 0.25f)) {
			takeoffSwitchStage(TAKEOFF_COMPLETED);
		}
		g_altitudeHoldLog.logTakeoffState = 20;
		break;

	case TAKEOFF_COMPLETED:
		/* Beep confirmation */
		beeperConfirmationBeeps(2);

		switchOnGroundDetectionState(NOT_ONGROUND);

		/* Store detected hover throttle and target altitude */
		if (takeoffStage_prev == TAKEOFF_DETECT_SONAR_CLIMB) { /* we completed during ascent phase */
			g_altHoldHoverThrottle = rcThrottleCommand  - TAKEOFF_THROTTLE_DELTA;
		}
		else if (takeoffStage_prev == TAKEOFF_DETECT_SONAR_DESCENT) { /* we completed during descent phase */
			g_altHoldHoverThrottle = rcThrottleCommand + TAKEOFF_THROTTLE_DELTA;
		}
		else { /* shouldn't get here */
			g_altHoldHoverThrottle = rcThrottleCommand;
			beeperConfirmationBeeps(3);
		}
		
		g_resetSnlVelEst = true; //do not reset 2 axis vel estimation if we use automatic takeoff
		int stickPos = getRelativeThrottleStickPosition(rcStickCommand[THROTTLE], 0);
		targetAglAltitude = scaleRange(constrain(stickPos, MIN_STICK_POSITION, MAX_STICK_POSITION), MIN_STICK_POSITION, MAX_STICK_POSITION, 0, altHoldSettings->sonar_max_alt);

		/* Reset Altitude Hold PID - it will take over on next iteration */
		resetAltHold();

		DISABLE_STATE(TAKE_OFF);    /* Disarm/rearm required */
				
		/* send output to motors (next loop Alt PID will take over) */
		rcCommand[THROTTLE] = (int16_t)(g_altHoldHoverThrottle + rcThrottleCommand) / 2.0f;
		g_altitudeHoldLog.logTakeoffState = 30;
		break;

	case TAKEOFF_FAILED:
		/* takeoff failed - disarm the drone */
		beeperConfirmationBeeps(1);
		mwDisarm();
		DISABLE_STATE(TAKE_OFF);    /* Disarm/rearm required */
		g_altitudeHoldLog.logTakeoffState = 40;
		break;

	case TAKEOFF_MANUALLY_ABORTED:
		/*Don't disarm or disable TAKE_OFF state so we can restart the takeoff without disarm-rearm cycle */
		beeperConfirmationBeeps(1);
		takeoffSwitchStage(TAKEOFF_IDLE);    /* Switch back to IDLE - bring motors to minimum and allow restart */
        rcCommand[THROTTLE] = escAndServoConfig->minthrottle;   /* override throttle to prevent throttle spike */
        g_altitudeHoldLog.logTakeoffState = 50;
		break;

	default:
	    g_altitudeHoldLog.logTakeoffState = 0;
		break;
	}

#ifdef DEBUG_TAKEOFF_HOVER_THROTTLE
    debug[0] = rcCommand[THROTTLE];
    debug[1] = g_altHoldHoverThrottle;
#endif

#ifdef DEBUG_TAKEOFF2
	debug[0] = imuAccelInEarthFrame.V.Z;
	debug[1] = takeoffStage;
	debug[3] = throttleSlewRate;
#endif
}

bool isTakeoffAndIdle(void)
{
    return  (STATE(TAKE_OFF) && (takeoffStage == TAKEOFF_IDLE));
}

bool isTakeoffInProgress(void)
{
    return (STATE(TAKE_OFF) && (takeoffStage != TAKEOFF_IDLE));
}

bool isTakeoffCompleted(void)
{
    return (takeoffStage == TAKEOFF_COMPLETED);
}


void applyAltHold(airplaneConfig_t *airplaneConfig, throttleStatus_e throttleStatus, uint32_t currentTime)
{
	if (STATE(FIXED_WING)) {
		/* do nothing */
	}
	else {
		if (STATE(TAKE_OFF)) {
			executeMultirotorTakeOff(currentTime);
		}
		else {
			executeMultirotorAltHold(throttleStatus, currentTime);
		}

#ifdef DEBUG_TAKEOFF2
debug[2] = STATE(TAKE_OFF) ? 1 : 0;
#endif
	}

    /* Log altitude target as it was used */
    g_altitudeHoldLog.logTargetAlt = targetAglAltitude;
}

/* not used
static int getClimbRateFromThrottleStickPosition(int16_t rcThrottle)  //cm/s
{
	return (getRelativeThrottleStickPosition(rcThrottle, altHoldSettings->alt_hold_deadband) - 500) * altHoldSettings->max_roc_rod / 500;
}
*/

static void adjustAltitudeFromStickInput(uint32_t currentTime)
{
    static bool emergencyDescentStarted = false;
	static uint32_t previousTime = 0;

	/* delta time between alt updates */
	float dT = (currentTime - previousTime) * 1e-6f;
	previousTime = currentTime;

	/* Limit dT to a 2 cycle delay (RX is updated at 50Hz rate for PPM receivers) */
	dT = MIN(dT, 0.1f);

	/* Use rcStickCommand here - we need actual pilot input */
    if (FLIGHT_MODE(ALTHOLD_DIRECT)) {
        int stickPos = getRelativeThrottleStickPosition(rcStickCommand[THROTTLE], 0);

        if (g_estAltitude.est.quality != EST_ALT_QUAL_DR || STATE(TAKE_OFF) ) {
            targetAglAltitude = scaleRange(constrain(stickPos, MIN_STICK_POSITION, MAX_STICK_POSITION), MIN_STICK_POSITION, MAX_STICK_POSITION, 0, altHoldSettings->sonar_max_alt);
            emergencyDescentStarted = false;

            /* square signal */
            #ifdef SQUARE_STEP_ALT_TUNING
            if (targetAglAltitude > 0.8f*altHoldSettings->sonar_max_alt) {
                if ((currentTime / 1000000) % 20 < 10) {
                    targetAglAltitude = 25;
                }
                else {
                    targetAglAltitude = 100;
                }
            }
            #endif
        }
        else { /* EST_ALT_QUAL_DR */
            // Make sure landing detection only works when throttle stick is below takeoff abort threshold
            if (rcStickCommand[THROTTLE] < ((rxConfig->mincheck + 0.5f * (lookupThrottleRCMid - rxConfig->mincheck)) - TAKEOFF_THROTTLE_HYSTERESIS)) {
                if (!emergencyDescentStarted) {
                    // We are entering emergency landing, reset landing detector
                    emergencyDescentStarted = true;
                    resetLandingDetector(currentTime);
                }

                // If it is obvious that we have landed - go to takeoff state
                if (isLandingDetected(currentTime)) {
                    emergencyDescentStarted = false;
                    takeoffSwitchStage(TAKEOFF_IDLE);
                    ENABLE_STATE(TAKE_OFF);
                    beeperConfirmationBeeps(3);
                }
            }
            else {
                emergencyDescentStarted = false;
            }
        
            uint32_t emergencyDescentTime = currentTime - g_estAltitude.est.qualityChangeTime;
            float emergencyROD;

            if (emergencyDescentTime >= S2US(altHoldSettings->emergency_descent_timeout)) {
                emergencyROD = altHoldSettings->max_roc_rod;
            }
            else {
                emergencyROD = scaleRangef(emergencyDescentTime, 0, S2US(altHoldSettings->emergency_descent_timeout), 0, altHoldSettings->max_roc_rod);
            }

            targetAglAltitude -= emergencyROD * dT;    /* Slow descent */
            targetAglAltitude = constrainf(targetAglAltitude,
                                           g_estAltitude.est.aglAltitude - TARGET_ALT_RANGE_BELOW_CM,
                                           g_estAltitude.est.aglAltitude + TARGET_ALT_RANGE_ABOVE_CM);
        }

        #ifdef ENABLE_GROUND_AVOIDANCE
        groundAvoidance(g_estAltitude.est.aglAltitude);
        #endif
    }
    else if (FLIGHT_MODE(ALTHOLD_ASSIST)) {
        // TODO
    }
}

// Callback from the estimator to indicate that AGL was reset
void signalAltitudeResetConditionToALTHOLD(void)
{
    if (FLIGHT_MODE(SNL_MODE)) {
        // If we are in SNL - reset the target altitude in PID LPF to avoid a jump
        filterResetPt1(&altHoldPID.targetAltLPFState, g_estAltitude.est.aglAltitude);
    }
    else if (FLIGHT_MODE(ALTHOLD_DIRECT)) {
        // Force update of target altitude
        adjustAltitudeFromStickInput(micros());
        filterResetPt1(&altHoldPID.targetAltLPFState, targetAglAltitude);
    }
}

void updateAltHoldState(uint32_t currentTime)
{
	/* Check if the pilot selected DIRECT nor ASSIST more on his TX */
	if ((IS_RC_MODE_ACTIVE(BOXAHDIRECT) || IS_RC_MODE_ACTIVE(BOXAHASSIST)) && FLIGHT_MODE(ANGLE_MODE)) {
		/* Neither DIRECT nor ASSIST flight mode was active previously */
		if (!FLIGHT_MODE(ALTHOLD_DIRECT) && !FLIGHT_MODE(ALTHOLD_ASSIST)) {
			deltaThrottleCommand = 0;
			resetAltHold();
		}

		/* Pilot requests ALTHOLD DIRECT mode */
		if (IS_RC_MODE_ACTIVE(BOXAHDIRECT)) {
			if (!FLIGHT_MODE(ALTHOLD_DIRECT)) {
				ENABLE_FLIGHT_MODE(ALTHOLD_DIRECT);
                DISABLE_FLIGHT_MODE(ALTHOLD_ASSIST);
			}
		}
		/* Pilot requests ALT ASSIST */
        /*
        // Make sure AAH is not active
		else if (IS_RC_MODE_ACTIVE(BOXAHASSIST)) {
			if (!FLIGHT_MODE(ALTHOLD_ASSIST)) {
				ENABLE_FLIGHT_MODE(ALTHOLD_ASSIST);
				DISABLE_FLIGHT_MODE(ALTHOLD_DIRECT);
			}
		}
        */

		/* Process throttle stick input */
		if (FLIGHT_MODE(ALTHOLD_DIRECT) || FLIGHT_MODE(ALTHOLD_ASSIST)) {
			/* Either DIRECT nor ASSIST flight mode is activated. Process throttle input to recalculate the altitude target */
			adjustAltitudeFromStickInput(currentTime);
		}
	}
	else {
		/* Neither DIRECT not ASSIST */
		DISABLE_FLIGHT_MODE(ALTHOLD_DIRECT);
		DISABLE_FLIGHT_MODE(ALTHOLD_ASSIST);
	}

#ifdef DEBUG_ON_GROUND_DETECTION
debug[0] = g_onGroundState*10.0f ;//+STATE(TAKE_OFF)*5.0f ; //+ (float)STATE(SMALL_ANGLE)*1.0f ;
#endif
			/* We can enable automatic take-off only when DIRECT altitude hold is activated AND drone is UNARMED and throttle stick is idle */
    if ((IS_RC_MODE_ACTIVE(BOXAHDIRECT) /*|| IS_RC_MODE_ACTIVE(BOXAHASSIST)*/) && FLIGHT_MODE(ANGLE_MODE)) {
        /* Machine is unarmed, reset altitude PIDs and process TAKE_OFF state */
        if (!STATE(TAKE_OFF)) {
            if (!ARMING_FLAG(ARMED) || (g_onGroundState == ONGROUND)) {
                resetEstimatedAltitude();
                resetAltHold();
                takeoffSwitchStage(TAKEOFF_IDLE);
                ENABLE_STATE(TAKE_OFF);
            }
            else if (takeoffStage != TAKEOFF_COMPLETED) {
                takeoffSwitchStage(TAKEOFF_COMPLETED);
            }
        }
    }
	else {
        // Make sure we indicate takeoff completed when not in altitude hold mode
        DISABLE_STATE(TAKE_OFF);
        if (takeoffStage != TAKEOFF_COMPLETED) {
            takeoffSwitchStage(TAKEOFF_COMPLETED);
        }
	}

}

int32_t altitudeHoldGetEstimatedAltitude(void)
{
    return g_estAltitude.est.aglAltitude;
}

int32_t altitudeHoldGetEstimatedClimbRate(void)
{
    return g_estAltitude.est.aglVelocity;
}

#endif
