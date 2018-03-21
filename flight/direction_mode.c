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

#include "config/config.h"
#include "config/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/direction_mode.h"
#include "flight/onground_detection.h"
#include "flight/pos_estimation.h"
#include "flight/altitudehold.h"



// AMIMON:
// This file is an implementation of direction mode

#define MAX_INCLINATION_IN_DIRECTION_MODE       300
#define MAX_INCLINATION_FOR_VEL_EST             600
#define MAX_VEL_CMS                             2100     /* 20 m/s = 72 kmh */
#define AIR_DENSITY                             1.225    /* kg/m^3 */

g_directionModeLogData_t g_directionModeLogData;

extern uint16_t cycleTime;

float g_estVelEarth[2];                   /* Earth coordinates. In absence of magnetometer power-on heading is considered North */
float g_estVelBody[2];                /* Body coordinates.  */
float g_estForwardVel = 0.0f;

static float stickDamping_B[2];			    /* velocity calibration for stick roll and pitch */
static float hoverRoll;
static float fastVelRoll;
static uint8_t  inHoverMode = 0;

/* Run-time parameters */
static float    droneMass; /* kg */
static float    yawToRollRatio;
static float 	extraRoll;
static float    stickExtraRoll;
static float    dirModeValidVel;
static float    yawStickFactor;

static rxConfig_t *rxConfig;
static pidProfile_t *pidProfile;

void configureDirectionMode(directionModeSettings_t * initialDirectionModeConfig, rxConfig_t * initialRxConfig, pidProfile_t * initialPidProfile)
{
    rxConfig = initialRxConfig;
    pidProfile = initialPidProfile;

    droneMass = GRAM_TO_KG(initialDirectionModeConfig->droneMass);
    extraRoll = (float) initialDirectionModeConfig->extraRoll; /* decidegree */
    stickExtraRoll = (float) initialDirectionModeConfig->stickExtraRoll / 100.f; /* percent stick */
    dirModeValidVel = initialDirectionModeConfig->dirModeValidVel;
    yawToRollRatio = initialDirectionModeConfig->yawToRollRatio;
    yawStickFactor = initialDirectionModeConfig->yawStickFactor;

}

void getStickDamping(float stickDamping[2])
{
    if (FLIGHT_MODE(ANGLE_MODE)) {
        float stickPosAil = (float)getRcStickDeflection(FD_ROLL, rxConfig->midrc);
        float stickPosEle = (float)getRcStickDeflection(FD_PITCH, rxConfig->midrc);

        stickDamping[0] = (500.0f - ABS(stickPosEle)) / 500.0f;  /* 1 at center pitch stick, 0 = max stick deflection */
        stickDamping[1] = (500.0f - ABS(stickPosAil)) / 500.0f;  /* 1 at center roll stick, 0 = max stick deflection */
    }
    else {
        stickDamping[0] = 0.0;
        stickDamping[1] = 0.0;
    }
}

void reset2AxisVelocityEstimate(void)
{
     g_estVelEarth[X] = 0.f;
     g_estVelEarth[Y] = 0.f;
     g_estVelBody[X]= 0.f;
     g_estVelBody[Y]= 0.f;
}

float calcDragAccelYb(float estRightVel )
{
    const float Cd0 = 0.01575f * 2.0f ; /*estimated sideways drag twice the size of forward drag*/
    const float K   = 0.062f;
    const float W   =  droneMass * CMS_TO_MS(GRAVITY_CMSS); /*mks*/

    static float rightVel_prev = 0.f;
    static float Cdi = 0.0f;

    float Cd0Factor = 0.0f;
    float dAccDrag = 0.0f;

    if (!ARMING_FLAG(ARMED)) {
        rightVel_prev = 0;
    }

    float rollAngle = constrainf(DECIDEGREES_TO_DEGREES(attitude.values.roll),
                                  -MAX_INCLINATION_FOR_VEL_EST,
                                  MAX_INCLINATION_FOR_VEL_EST);

    /* Apply aerodynamic drag */
    if ((ABS(estRightVel) >= 300) && (ABS(estRightVel) <= ABS(rightVel_prev))) { /*if we have enough velocity and it's decreasing */
        Cdi = K * 4.f * sq(W) / ( (sq(AIR_DENSITY) * sq(sq(CMS_TO_MS(estRightVel)))) );
    }
    else {
        Cdi = 0.f;
    }
    Cd0Factor = constrainf(-0.08f * sq(rollAngle) + 3.f, 1.f, 3.f); //TODO - fix for large angles
    dAccDrag = SIGN(estRightVel) * 0.5 * AIR_DENSITY * sq(CMS_TO_MS(estRightVel)) * (Cd0 * Cd0Factor + Cdi) / droneMass;
	
	rightVel_prev = estRightVel;
	
    return MS_TO_CMS(dAccDrag);
}

float calcDragAccelXb(float estForwardVel )
{
    const float Cd0 = 0.01575f;
    const float K   = 0.062f;
    const float W   =  droneMass * CMS_TO_MS(GRAVITY_CMSS); /*mks*/

    static float forwardVel_prev = 0.f;
    static float Cdi = 0.0f;
  
    float Cd0Factor = 0.0f;
    float dAccDrag = 0.0f;

    if (!ARMING_FLAG(ARMED)) {
        forwardVel_prev = 0.f;
    }

    float pitchAngle = constrainf(DECIDEGREES_TO_DEGREES(attitude.values.pitch),
                                  -MAX_INCLINATION_FOR_VEL_EST,
                                  MAX_INCLINATION_FOR_VEL_EST);
 
    /* Apply aerodynamic drag */
    if ((ABS(estForwardVel) >= 300) && (ABS(estForwardVel) <= ABS(forwardVel_prev))) { /*if we have enough velocity and it's decreasing */
        Cdi = K * 4.f * sq(W) / ((sq(AIR_DENSITY) * sq(sq(CMS_TO_MS(estForwardVel)))));
    }
    else {
        Cdi = 0.f;
    }
    Cd0Factor = constrainf(-0.08f * sq(pitchAngle) + 3.f, 1.f, 3.f); //TODO - fix for large angles
    dAccDrag = SIGN(estForwardVel) * 0.5 * AIR_DENSITY * sq(CMS_TO_MS(estForwardVel)) * (Cd0 * Cd0Factor + Cdi) / droneMass ;
	
	forwardVel_prev = estForwardVel;
	
    return MS_TO_CMS(dAccDrag);
}

void update2AxisVelocityEstimate(void)
{
	
	    /* reset velocity estimator according to estimated altitude when in ACRO */
    if (!FLIGHT_MODE(ALTHOLD_ASSIST | ALTHOLD_DIRECT) && !g_resetSnlVelEst) {
		
		reset2AxisVelocityEstimate();
       
	   //TODO : replace 1500 with hover throttle 
	   if (g_estAltitude.est.aglAltitude >= 50.f || rcCommand[THROTTLE] > 1500 ) {

            g_resetSnlVelEst = true;
        }
    }
	
    if (!isImuReady() || !ARMING_FLAG(ARMED) || (g_onGroundState == ONGROUND)|| STATE(TAKE_OFF) ) {
        reset2AxisVelocityEstimate();
		g_resetSnlVelEst = false;
	}
       
    float dT = cycleTime * 1e-6;

    t_fp_vector thrustAccelVector;
 

    /* Here thrust vector is in body frame */
    // What we know is that vertical acceleration is composed of Gravity and vertical component of Thrust:
    // AccZ_Earth = ThrustZ_Earth - Gravity =  Thrust * cosZ - Gravity (where cosZ is cosine of body tilt)
    // In hover or stable forward flight AccZ_Earth = 0
    // From these: Thrust * cosZ = (AccZ_Earth_IMU + Gravity)

    float cosZConstrained = calculateCosTiltAngle(); /* Zero Protection */
	
	/* limit for 60 deg bank */ /*TODO modify for acro use */
    if (cosZConstrained < 0.0f) {
        cosZConstrained = constrainf(cosZConstrained, -1.0f, -0.5f);
    }
    if (cosZConstrained >= 0.0f) {
        cosZConstrained = constrainf(cosZConstrained, 0.5f, 1.0f);
    }

    /* Thrust is always on Z axis in body frame */
    thrustAccelVector.V.X = 0;
    thrustAccelVector.V.Y = 0;
    thrustAccelVector.V.Z = (imuAccelInEarthFrame.V.Z + GRAVITY_CMSS) / cosZConstrained;

#ifdef DEBUG_DIR_ACCEL
    debug[2] = imuAccelInEarthFrame.V.Z;
    debug[3] = thrustAccelVector.V.Z;
#endif

    /* Transform to Earth frame - now we have N-E components of thrust-induced acceleration */
    imuTransformVectorBodyToEarth(&thrustAccelVector);

    /* Integrate global acceleration to yield global velocity */
    g_estVelEarth[X] = g_estVelEarth[X] + thrustAccelVector.V.X  * dT;
    g_estVelEarth[Y] = g_estVelEarth[Y] + thrustAccelVector.V.Y  * dT;

	/* 2D Rotation velocity from Earth to Body frame*/
    rot2D_E2B(g_estVelEarth, attitude.values.yaw, g_estVelBody);

	/* Add friction in Body frame */
	g_estVelBody[X] -= calcDragAccelXb(g_estVelBody[X]) * dT ;
    g_estVelBody[Y] -= calcDragAccelYb(g_estVelBody[Y]) * dT ;

	 /* Apply global constraint to velocity */
    g_estVelBody[X] = constrainf(g_estVelBody[X], -MAX_VEL_CMS, MAX_VEL_CMS);
    g_estVelBody[Y] = constrainf(g_estVelBody[Y], -MAX_VEL_CMS, MAX_VEL_CMS);
	
    /* Rotate velocity from Body to Earth frame */
    rot2D_B2E(g_estVelBody, attitude.values.yaw,g_estVelEarth);
	
    g_directionModeLogData.logSnlFwdVel = g_estVelBody[X];
	g_directionModeLogData.logSnlRightVel = g_estVelBody[Y];

#ifdef DEBUG_PANIC_MODE
//debug[3] = sqrt(sq(g_estVelBody[X])+sq(g_estVelBody[Y])); 
#endif

#ifdef DEBUG_GPS_VS_2_AXIS
debug[0] = sqrt(sq(g_estVelBody[X])+sq(g_estVelBody[Y]));        
debug[1] = GPS_speed;         
debug[2] = (GPS_ground_course);  

#endif

#ifdef DEBUG_GPS_VS_2_AXIS_EARTH
    debug[0] = g_estVelEarth[X];         // est vel north
    debug[1] = g_estVelEarth[Y];         // est vel east
	
	debug[2] = (GPS_speed) * cos_approx(DECIDEGREES_TO_RADIANS(GPS_ground_course));  // V_N
    debug[3] = (GPS_speed) * sin_approx(DECIDEGREES_TO_RADIANS(GPS_ground_course));  // V_E
#endif
}

void reset1AxisVelocityEstimate(void)
{
     g_estForwardVel = 0.f;
}

void update1AxisVelocityEstimate(void)
{
    const float Cd0 = 0.01575f;
    const float K   = 0.062f;
    const float W   =  droneMass * CMS_TO_MS(GRAVITY_CMSS); /*mks*/

    static float forwardVel_prev = 0.f;
    static float Cdi = 0.0f;

    float dT = cycleTime * 1e-6;
    float dV = 0.0f;
    float Cd0Factor = 0.0f;
    float dVDrag = 0.0f;

    if (!isImuReady() || !ARMING_FLAG(ARMED) || (g_onGroundState == ONGROUND)|| STATE(TAKE_OFF) ) {
        reset1AxisVelocityEstimate();
    }

    float pitchAngle = DECIDEGREES_TO_DEGREES(constrain(attitude.values.pitch,
                                              -MAX_INCLINATION_IN_DIRECTION_MODE,
                                               MAX_INCLINATION_IN_DIRECTION_MODE));
    float pitchAngleRad = DEGREES_TO_RADIANS(pitchAngle);
    float tanPitchAngle = tan_approx(pitchAngleRad);

    /* Apply forward acceleration */
    dV = GRAVITY_CMSS * tanPitchAngle * dT;
    /* Apply aerodynamic drag */
    if ((ABS(g_estForwardVel) >= 300) &&
        (ABS(g_estForwardVel) <= ABS(forwardVel_prev))) { /*if we have enough velocity and it's decreasing */
        Cdi = K * 4.f * sq(W) / ((sq(AIR_DENSITY) * sq(sq(CMS_TO_MS(g_estForwardVel)))));
    }
    else {
        Cdi = 0.f;
    }
    Cd0Factor = constrainf(-0.08f * sq(pitchAngle) + 3.f, 1.f, 3.f);
    dVDrag = SIGN(g_estForwardVel) * 0.5 * AIR_DENSITY * sq(CMS_TO_MS(g_estForwardVel)) * (Cd0 * Cd0Factor + Cdi) / droneMass * dT;
    dV -= MS_TO_CMS(dVDrag);

    forwardVel_prev = g_estForwardVel;
    g_estForwardVel += dV;
    g_estForwardVel = constrainf(g_estForwardVel, -MAX_VEL_CMS, MAX_VEL_CMS);

    g_directionModeLogData.logDirModeVel = g_estForwardVel;

#ifdef DEBUG_DIR_VELOCITY
    debug[1] = g_estForwardVel;
    debug[2] = 1000.0f*dV;
    debug[3] = 100.0f*tanPitchAngle;
#endif

#ifdef DEBUG_GPS_VS_VEL
    debug[0] = cos_approx(DECIDEGREES_TO_RADIANS(GPS_ground_course));                                                      // est vel fwd
    debug[1] = GPS_speed;// * cos_approx(DECIDEGREES_TO_RADIANS(GPS_ground_course));  // V_N
    debug[2] = 1000.0f*dV;
    debug[3] = 100.0f*stickDamping_B[1];
#endif

#ifdef DEBUG_DRAG_MODEL
    debug[0] = 1000*dVDrag;
    debug[1] = GPS_speed;
    debug[2] = 1000.f*Cdi;
    debug[3] = 1000.*dV;
#endif
}

void updateDirectionModeState(void)
{
    if (IS_RC_MODE_ACTIVE(BOXDIRECTION_YAW) && !STATE(FIXED_WING) && FLIGHT_MODE(ANGLE_MODE)) {
        ENABLE_FLIGHT_MODE(DIRECTION_MODE);
    }
    else {
        DISABLE_FLIGHT_MODE(DIRECTION_MODE);
    }
}

void applyDirectionModeController(controlRateConfig_t * controlRateConfig)
{
    float controlWeight;
    float targetYawRate;
    float targetRollAngle;

    /* Direction mode - automatically apply yaw on roll (coordinated turn), calculate centripetal acceleration to fly balanced curve */
    //if (estBodyForwardVel >= MIN_VEL_CMSS && rcCommand[PITCH] >= 0) {
    // if (estBodyForwardVel >= dirModeMinVel) {
    //    controlWeight = constrainf((estBodyForwardVel - dirModeMinVel) / (dirModeValidVel - dirModeMinVel), 0.0f, 1.0f);
    // Ideal banked turn follow the equations:
    //      forward_vel^2 / radius = Gravity * tan(roll_angle)
    //      yaw_rate = forward_vel / radius
    // If we solve for roll angle we get:
    //      tan(roll_angle) = forward_vel * yaw_rate / Gravity
    // If we solve for yaw rate we get:
    //      yaw_rate = tan(roll_angle) * Gravity / forward_vel

    /* Apply generalized friction */
    getStickDamping(stickDamping_B);

    int16_t rcCommandYaw = (int16_t) (yawStickFactor * rcStickCommand[ROLL]); /* Roll stick controls yaw rate - TODO modify factor */

    targetYawRate = DEGREES_TO_RADIANS(rcCommandToAxisRate(controlRateConfig->rates[YAW], rcCommandYaw));

    if (ABS(g_estForwardVel) <= dirModeValidVel) {
        hoverRoll = constrainf(extraRoll / stickExtraRoll * SIGN(rcStickCommand[ROLL]) * (1.0 - stickDamping_B[1]), -extraRoll, extraRoll);
        fastVelRoll = 0.0f;
        controlWeight = 0.3f; //constrainf(1.0f - (dirModeValidVel - ABS(g_estForwardVel)) / dirModeValidVel, 0.3f, 1.0f);
        inHoverMode = 1;
    }
    else {
        hoverRoll = 0.0f;
        fastVelRoll = RADIANS_TO_DECIDEGREES(atan2_approx(g_estForwardVel * targetYawRate * yawToRollRatio, GRAVITY_CMSS));
        controlWeight = 1.0f;
        inHoverMode = 0;
    }

    targetRollAngle = SIGN(g_estForwardVel) * fastVelRoll + hoverRoll;

    rcCommand[ROLL] = angleDecidegreesToRcCommand(pidProfile, FD_ROLL,
                                                  constrainf(targetRollAngle,
                                                  -MAX_INCLINATION_IN_DIRECTION_MODE,
                                                  MAX_INCLINATION_IN_DIRECTION_MODE));
    rcCommand[YAW] += -controlWeight * rcCommandYaw;

    g_directionModeLogData.targetRollAngle = constrainf(targetRollAngle * yawToRollRatio,
                                                       -MAX_INCLINATION_IN_DIRECTION_MODE,
                                                        MAX_INCLINATION_IN_DIRECTION_MODE); // deg*10
    g_directionModeLogData.targetYawRate = RADIANS_TO_DEGREES(targetYawRate);   // dps

    rcCommand[YAW] = constrain(rcCommand[YAW], -500, +500);

#ifdef DEBUG_DIR_ROLL_TGT
    debug[0] = SIGN(g_estForwardVel) * targetRollAngle * 100;
    debug[1] = hoverRoll * 100;
    debug[2] = RADIANS_TO_DEGREES(targetYawRate);
    debug[3] = controlWeight * 100;
#endif

#ifdef DEBUG_DIR_COMMAND
    debug[0] = g_estVelBody[X];
    debug[1] = controlWeight * 100;
    debug[2] = rcCommand[ROLL];
    debug[3] = rcCommand[YAW];
#endif
}


uint8_t isDroneHovering(void)
{
    return inHoverMode;
}

