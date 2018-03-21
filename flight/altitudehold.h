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
#include "flight/mixer.h"

#include "sensors/barometer.h"

#define CRASH_ACCEL_EARTH_Z_CMSS               1500
#define THROTTLE_PANIC_MODE_RC_COMMAND_DELTA   30

#define TAKEOFF_PROP_GUARD_MARGIN              55
#define TAKEOFF_THROTTLE_ERROR_MARGIN          20
#define TAKEOFF_THROTTLE_DELTA                 10
#define TAKEOFF_THROTTLE_SLEW_RATE             7
#define TAKEOFF_END_ALTITUDE_STD_CM            15
#define TAKEOFF_TIMEOUT_CLIMB_SEC              2.7f
#define TAKEOFF_TIMEOUT_DESCENT_SEC            3.5f
#define TAKEOFF_THROTTLE_HYSTERESIS            10

#define MIN_LAND_THROTTLE_SLEW_RATE            2.0f
#define MAX_LAND_THROTTLE_SLEW_RATE            7.0f


typedef struct {
    int16_t     sonar_pitch_angle;
    uint8_t     alt_hold_deadband;
    uint16_t    hover_throttle;
    uint16_t    sonar_max_alt;
    uint16_t    baro_max_alt;
    uint16_t    max_roc_rod;
    uint16_t    max_snl_rod;
    uint16_t    sonar_range;        // sonar's considered good up to sonar_range
    float       alt_baro_z_w;
    float       alt_baro_v_w;
    float       alt_sonar_z_w;
    float       alt_sonar_v_w;
    float       alt_baro_accz_bias_w;
    float       emergency_descent_timeout;
} altitudeHoldSettins_t;

typedef struct {
    int16_t logEstAglAlt;
    int16_t logEstAglVel;
    int16_t logTargetAlt;
    int16_t logTargetVel;

    int16_t logPidP;
    int16_t logPidI;
    int16_t logPidD;
    int16_t logPidF;

    int16_t sonarConsistency;
    int16_t estAltQuality;
    int16_t altHoldAdjustment;
    int16_t altHoldAccZ;
    int16_t estBaroOffset;
    int16_t debugTelemetyValues[4];

    int16_t logTakeoffState;
} altitudeHoldLogData_t;

extern altitudeHoldLogData_t g_altitudeHoldLog;

void configureAltitudeHoldPID(void);
void configureAltitudeHold(pidProfile_t *initialPidProfile,
                           altitudeHoldSettins_t * initialAltHoldSettings,
                           escAndServoConfig_t *initialEscAndServoConfig,
                           rxConfig_t * initialRxConfig);
void applyAltHold(airplaneConfig_t *airplaneConfig, throttleStatus_e throttleStatus, uint32_t currentTime);
void updateAltHoldState(uint32_t currentTime);

int32_t altitudeHoldGetEstimatedAltitude(void);
int32_t altitudeHoldGetEstimatedClimbRate(void);

bool isTakeoffAndIdle(void);
bool isTakeoffInProgress(void);
bool isTakeoffCompleted(void);

extern void signalAltitudeResetConditionToALTHOLD(void);
extern void signalAltitudeResetConditionToSNL(void);

extern void resetLandingDetector(uint32_t currentTime);
extern bool isLandingDetected(uint32_t currentTime);

extern int16_t g_altHoldHoverThrottle;
