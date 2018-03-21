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

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/pos_estimation.h"
#include "flight/onground_detection.h"

#include "config/config.h"
#include "config/runtime_config.h"

onGroundState_e g_onGroundState = UNDETERMINED_ONGROUND;

static uint32_t lastTimeRatesNotFIxed = 0;
static uint32_t lastTimeRotatingFast = 0;
static uint32_t lastTimeAccelerating = 0;
static uint32_t lastTimeIsMotorStop = 0;
 
#define ONGROUND_RATE_FIXED_UPDATE_HZ    50 //Hz  
#define ONGROUND_RATES_FIXED_TIME_US     100 
#define ONGROUND_DETECTION_TIME_US       250     // if no movement for 0.5 sec, we are on the ground
#define ZERO_RATE_THRESHOLD_DDPS         150     // 1.5 deg per second is considered zero
#define ZERO_ACCEL_EF_Z_CMSS        	 25      // 0.025G
#define FIXED_DIFF_THRESHOLD_DDPS        15  

void switchOnGroundDetectionState(onGroundState_e newState)
{
    g_onGroundState = newState;
}

void resetOnGroundDetection(void) {
	lastTimeRotatingFast = 0;
	lastTimeAccelerating = 0;
	lastTimeIsMotorStop = 0;

	switchOnGroundDetectionState(UNDETERMINED_ONGROUND);
}

// record last time drone was rotating

void updateOnGroundRatesAreStable (float rpyActualRates[3],uint32_t currentTime)
{ 
	int axis;
	static uint32_t previousTimeUpdate = 0;
	static float rpyPrevRates[3] = { 0 };


	if ((currentTime - previousTimeUpdate) < HZ2US(ONGROUND_RATE_FIXED_UPDATE_HZ)) {
		return;
	}

    previousTimeUpdate = currentTime;
		

	if (    ( ABS(rpyPrevRates[0] - rpyActualRates[0] ) > FIXED_DIFF_THRESHOLD_DDPS ) ||
			( ABS(rpyPrevRates[1] - rpyActualRates[1] ) > FIXED_DIFF_THRESHOLD_DDPS ) || 
			( ABS(rpyPrevRates[2] - rpyActualRates[2] ) > FIXED_DIFF_THRESHOLD_DDPS ) ) {
			lastTimeRatesNotFIxed = currentTime;
	}
	
#ifdef DEBUG_ON_GROUND_DETECTION
debug[2] = (float)ABS(rpyPrevRates[0] - rpyActualRates[0] )*100.0f;
#endif
	
	for (axis = 0; axis < 3; axis++) {
		rpyPrevRates[axis] = rpyActualRates[axis];
		}
	
}

void updateOnGroundDetectionRates(float rpyActualRates[3],uint32_t currentTime)
{ 
	int axis;
#ifdef DEBUG_ON_GROUND_DETECTION
//debug[2] = (float)(ABS(rpyActualRates[0])+ABS(rpyActualRates[1])+ABS(rpyActualRates[2]))*100.0f;
#endif
	for (axis = 0; axis < 3; axis++) {
		if (ABS(rpyActualRates[axis]) > ZERO_RATE_THRESHOLD_DDPS) {
			lastTimeRotatingFast = currentTime;
		}
	}
}

void updateOnGroundDetectionAcceleration(uint32_t currentTime)
{
#ifdef DEBUG_ON_GROUND_DETECTION
debug[3] = (float)ABS(imuAccelInEarthFrame.V.Z)*100.0f;
#endif

	if (ABS(imuAccelInEarthFrame.V.Z) > ZERO_ACCEL_EF_Z_CMSS ) {
	   lastTimeAccelerating = currentTime;
	}
}

void updateOnGroundDetection(uint32_t currentTime)
{
		
	if (!isImuReady()) {
		switchOnGroundDetectionState(ONGROUND);
		return;
	}

	if( STATE(MOTOR_STOP) &&
	    ( currentTime - lastTimeRatesNotFIxed ) > ONGROUND_DETECTION_TIME_US  &&
		( currentTime - lastTimeRotatingFast  ) > ONGROUND_DETECTION_TIME_US  &&
		( currentTime - lastTimeAccelerating  ) > ONGROUND_DETECTION_TIME_US  ) {
		switchOnGroundDetectionState(ONGROUND);
	}
	else {
		switchOnGroundDetectionState(UNDETERMINED_ONGROUND);
	}
			
#ifdef DEBUG_ON_GROUND_DETECTION
debug[1] = (float)(( currentTime - lastTimeRatesNotFIxed   ) > ONGROUND_DETECTION_TIME_US)*5.0f + (float)(STATE(MOTOR_STOP))*10.0f + (float)((currentTime - lastTimeRotatingFast) > ONGROUND_DETECTION_TIME_US)*2.0f + (float)((currentTime - lastTimeAccelerating) > ONGROUND_DETECTION_TIME_US)*1.0f;
#endif
}
