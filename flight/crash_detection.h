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

/* typedef enum {
    CRASH_DETECT_ALL_MODES = 0,

    CRASH_DETECT_MODES_COUNT
} crashDetectionMode_e;
 
typedef struct {
    crashDetectionMode_e    mode;
} crashDetectionSettings_t; */

typedef enum {
	NOT_CRASHED = 0,
    GROUND_IMPACT_LIGHT,
	GROUND_IMPACT_MID,
	GROUND_IMPACT_HARD,
    ALL_AXIS_CRASH,

} crashState_e;
 
/* void configureCrashDetection(crashDetectionSettings_t * initialCrashDetectionSettings);

void updateCrashDetectionRates(float rpyRequestedRates[3], float rpyActualRates[3], float rpyRateErrors[3]);
void updateCrashDetectionAngles(float rpRequestedAttitude[2]);
void updateCrashDetectionAcceleration(uint32_t currentTime);

void updateCrashDetection(uint32_t currentTime);

void setMotorStopState(bool state);
void setZVel(float velocity); */

void updateGroundImpact(uint32_t currentTime);
bool crashResetEstimatedAltitude(void);

extern int16_t g_logCrashDetectLevel;
extern crashState_e g_crashState;
extern bool g_snlDisableCrashDetection;

