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

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/altitudehold.h"
#include "flight/direction_mode.h"
#include "flight/pos_estimation.h"
#include "flight/crash_detection.h"
#include "flight/onground_detection.h"
#include "flight/panic_mode.h"

#include "config/config.h"
#include "config/runtime_config.h"

#define BARO_UPDATE_HZ                      35  /* 35Hz */
#define SONAR_UPDATE_HZ                     10  /* 10Hz */

//#define USE_SONAR_TILT_COMPENSATION

//#define USE_SIMULATED_OUTLIER_GENERATION    5       // 5% chance of an outlier

#define SONAR_ACCEPTANCE_RANGE_CM           50
#define RAW_SONAR_MIN_RANGE                 5           // Nothing <5cm is valid
#define SONAR_CONSISTENCY_RC_CONSTANT       0.47802f    // Time-constant for discrete-time implementation of RC-filter (https://en.wikipedia.org/wiki/Low-pass_filter#RC_filter)
#define SONAR_CONSISTENCY_ALPHA             ((1.f / SONAR_UPDATE_HZ) / ((1.f / SONAR_UPDATE_HZ) + SONAR_CONSISTENCY_RC_CONSTANT))
#define SONAR_CONSISTENCY_SHORTCUT_SAMPLES  4
#define SONAR_IN_TAKEOFF_RANGE_SAMPLES      2

/* These two values should obey the following loose rules:
 *  - They should be high enough to reliably suppress jitter
 *  - They should be high enough to reliably produce median value
 *  - They should be low enough to introduce minimal delay  (350ms is low enough)
 *  - They should introduce roughly the same delay in baro and sonar (SONAR_MOVING_AVERAGE_LENGTH / SONAR_UPDATE_HZ ~ BARO_MOVING_AVERAGE_LENGTH / BARO_UPDATE_HZ)
 */
#define SONAR_MOVING_AVERAGE_LENGTH         5
#define BARO_MOVING_AVERAGE_LENGTH          18

/* Parameters for weight boost after crash for faster convergence */
#define WEIGHT_BOOST_ALPHA                  (1.0f - (0.001 / ((1.0f / (2.0f * M_PIf * 0.33f)) + 0.001)))     // 0.33Hz filter at 1kHz loop rate
#define WEIGHT_BOOST_SONAR                  3.0f
#define WEIGHT_BOOST_BARO                   6.0f

/* What good/bad/outlier measurements contribute to consistency */
#define GOOD_MEASUREMENT_CONSISTENCY        1.0f
#define OUTLIER_MEASUREMENT_CONSISTENCY     0.0f
#define BAD_MEASUREMENT_CONSISTENCY         0.0f

estAltitude_s   g_estAltitude;
bool g_resetSnlVelEst = false;


// FIXME
extern altitudeHoldSettins_t * altHoldSettings;
extern engageSnlFlightMode_e g_engageSnlFlightMode;

static bool allowSonarForAglCorrection(void)
{

    if (FLIGHT_MODE(ALTHOLD_DIRECT | ALTHOLD_ASSIST)) {
        return true;

    }
    else if (FLIGHT_MODE(SNL_MODE) &&
           ((g_engageSnlFlightMode == FLM_SHIELD) || ((g_engageSnlFlightMode == FLM_REGULAR) && (g_snlState == SNL_LAND)))) {
        return true;
    }
    else {

        return false;
    }
}

#ifdef USE_SONAR_TILT_COMPENSATION
static float calcCosTiltInEF(float pitchRadBF)
{
	t_fp_vector vector;
	vector.V.X = -sin_approx(pitchRadBF);
	vector.V.Y =0.0f;
	vector.V.Z = cos_approx(pitchRadBF);

	/* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
	imuTransformVectorBodyToEarth(&vector);
	return vector.V.Z;  /* now its cos(tilt_angle) in EF */
}
#endif

static void updateSonarReadings(uint32_t currentTime)
{
    static float sonarBuffer[SONAR_MOVING_AVERAGE_LENGTH] = { 0 };
    static int sonarBufferIdx = 0;
    static int sonarBufferInitialized = 0;
    static uint32_t previousTimeUpdate = 0;
    static uint32_t goodSamplesInARow = 0;
	static uint32_t goodSamplesInTakeoffRange = 0;

	if ((currentTime - previousTimeUpdate) < HZ2US(SONAR_UPDATE_HZ)) {
		return;
	}

    previousTimeUpdate = currentTime;

    if (!sonarBufferInitialized) {
        int i;
        for (i = 0; i < SONAR_MOVING_AVERAGE_LENGTH; i++) {
            sonarBuffer[i] = 0;
        }

        g_estAltitude.sonar.meanAltitude = 0;
        sonarBufferInitialized = 1;
    }

    if (sensors(SENSOR_SONAR)) {
        float measurementSonarConsistency = 0;
#ifdef USE_SONAR_TILT_COMPENSATION
		const float rawSonarAlt = sonarRead();
		const float cosTiltSonarAngle = calcCosTiltInEF(DEGREES_TO_RADIANS(altHoldSettings->sonar_pitch_angle));
        float newSonarAlt = sonarCalculateAltitude(rawSonarAlt, cosTiltSonarAngle);
#else
        float newSonarAlt = sonarRead();
#endif

#ifdef USE_SIMULATED_OUTLIER_GENERATION
        if (newSonarAlt > 0 && (rand() % 100) <= USE_SIMULATED_OUTLIER_GENERATION) {
            newSonarAlt = newSonarAlt + 100;
        }
#endif

        // mainly for OSD use
        g_estAltitude.sonar.sonarRaw = newSonarAlt;

        if (newSonarAlt >= RAW_SONAR_MIN_RANGE && newSonarAlt <= altHoldSettings->sonar_range) {
            /* Update reading */
            g_estAltitude.sonar.currentAltitude = newSonarAlt;

			/* count number of sonar readings within takeoff expected range */
			
			if( isTakeoffInProgress() ) {
				if ( g_estAltitude.sonar.currentAltitude >= 20.0f && g_estAltitude.sonar.currentAltitude <= 60.0f) {
					goodSamplesInTakeoffRange++;
				}
				else {
					goodSamplesInTakeoffRange = 0;
				}
			} 
			else {
				goodSamplesInTakeoffRange = 0;
			}
			
            /* Process the outlier detection:
             *      If a sonar reading is above previous 5-point median by over SONAR_ACCEPTANCE_RANGE_CM it's considered an outlier
             *      Possible outliers are still consumed by mean calculation, so we'll be able to filter only a few consecutive outliers
             */
            if ((g_estAltitude.sonar.currentAltitude - g_estAltitude.sonar.meanAltitude) > SONAR_ACCEPTANCE_RANGE_CM) {
                g_estAltitude.sonar.isOutlier = true;
                g_estAltitude.sonar.sonarOutlierCount++;
                measurementSonarConsistency = OUTLIER_MEASUREMENT_CONSISTENCY;
                goodSamplesInARow = 0;
            }
            else {
                g_estAltitude.sonar.isOutlier = false;
                if (g_estAltitude.sonar.sonarOutlierCount > 0) {
                    g_estAltitude.sonar.sonarOutlierCount--;
                }
                measurementSonarConsistency = GOOD_MEASUREMENT_CONSISTENCY;
                goodSamplesInARow++;
            }

            /* Update mean altitude */
            g_estAltitude.sonar.meanAltitude = g_estAltitude.sonar.meanAltitude
                                                    - sonarBuffer[sonarBufferIdx] / SONAR_MOVING_AVERAGE_LENGTH
                                                    + g_estAltitude.sonar.currentAltitude / SONAR_MOVING_AVERAGE_LENGTH;
            sonarBuffer[sonarBufferIdx] = g_estAltitude.sonar.currentAltitude;
            if (++sonarBufferIdx >= SONAR_MOVING_AVERAGE_LENGTH) {
                sonarBufferIdx = 0;
            }

            /* Indicate sonar update */
            g_estAltitude.sonar.isUpdated = true;
            g_estAltitude.sonar.deltaTime = US2S(currentTime - g_estAltitude.sonar.lastUpdateTime);
            g_estAltitude.sonar.lastUpdateTime = currentTime;
        }
        else {
            goodSamplesInARow = 0;
            measurementSonarConsistency = BAD_MEASUREMENT_CONSISTENCY;
        }

        /* Update sonar consistency */
        if ( (goodSamplesInARow >= SONAR_CONSISTENCY_SHORTCUT_SAMPLES || 
		      goodSamplesInTakeoffRange >= SONAR_IN_TAKEOFF_RANGE_SAMPLES ) &&
              g_estAltitude.sonar.sonarConsistency < SONAR_CONSISTENCY_HIGH_THRESHOLD ) {
            g_estAltitude.sonar.sonarConsistency = SONAR_CONSISTENCY_HIGH_THRESHOLD;
        }
        else {
            g_estAltitude.sonar.sonarConsistency = g_estAltitude.sonar.sonarConsistency * (1.0f - SONAR_CONSISTENCY_ALPHA)
                                                        + measurementSonarConsistency * SONAR_CONSISTENCY_ALPHA;
        }
    }
    else {
        g_estAltitude.sonar.isUpdated = false;
        g_estAltitude.sonar.lastUpdateTime = 0;
        g_estAltitude.sonar.currentAltitude = 0;
        g_estAltitude.sonar.meanAltitude = 0;
        g_estAltitude.sonar.sonarConsistency = 0;
        sonarBufferInitialized = 0; // Force mean re-init
    }
}

static void updateBaroReadings(uint32_t currentTime)
{
    static float baroBuffer[BARO_MOVING_AVERAGE_LENGTH] = { 0 };
    static int baroBufferIdx = 0;
    static int baroBufferInitialized = 0;
    static uint32_t previousTimeUpdate = 0;

	if ((currentTime - previousTimeUpdate) < HZ2US(SONAR_UPDATE_HZ)) {
		return;
	}

    previousTimeUpdate = currentTime;

    if (!baroBufferInitialized) {
        int i;
        for (i = 0; i < SONAR_MOVING_AVERAGE_LENGTH; i++) {
            baroBuffer[i] = 0;
        }

        g_estAltitude.baro.meanAltitude = 0;
        baroBufferInitialized = 1;
    }

	if (sensors(SENSOR_BARO)) {
		g_estAltitude.baro.currentAltitude = baroCalculateAltitude();

		if (isBaroCalibrationComplete()) {
            g_estAltitude.baro.isUpdated = true;
            g_estAltitude.baro.deltaTime = US2S(currentTime - g_estAltitude.baro.lastUpdateTime);
            g_estAltitude.baro.lastUpdateTime = currentTime;
		}


        /* Calculate mean altitude */
        g_estAltitude.baro.meanAltitude = g_estAltitude.baro.meanAltitude
                                            - baroBuffer[baroBufferIdx] / BARO_MOVING_AVERAGE_LENGTH
                                            + g_estAltitude.baro.currentAltitude / BARO_MOVING_AVERAGE_LENGTH;
        baroBuffer[baroBufferIdx] = g_estAltitude.baro.currentAltitude;
        if (++baroBufferIdx >= BARO_MOVING_AVERAGE_LENGTH) {
            baroBufferIdx = 0;
        }
	}
	else {
        g_estAltitude.baro.isUpdated = false;
        g_estAltitude.baro.lastUpdateTime = 0;
        g_estAltitude.baro.currentAltitude = 0;
        g_estAltitude.baro.meanAltitude = 0;
        baroBufferInitialized = 0;
	}
}

static void updateIMUReadings(uint32_t currentTime)
{
    g_estAltitude.imu.deltaTime = US2S(currentTime - g_estAltitude.imu.lastUpdateTime);
    g_estAltitude.imu.lastUpdateTime = currentTime;
    g_estAltitude.imu.correctedAccZ = imuAccelInEarthFrame.V.Z + g_estAltitude.imu.biasAccZ;
}

void resetEstimatedAltitude(void)
{
    g_estAltitude.est.aglAltitude = 0;
    g_estAltitude.est.aglVelocity = 0;
    g_estAltitude.est.baroGroundOffset = g_estAltitude.baro.meanAltitude;
    g_estAltitude.est.qualityChangeTime = 0;

    g_estAltitude.sonar.sonarConsistency = 0;
}

static void resetEstimatedAltitudeToSonar(void)
{
    // Reset altitude and baro ground offset
    if (g_estAltitude.sonar.sonarConsistency >= SONAR_CONSISTENCY_LOW_THRESHOLD && allowSonarForAglCorrection()) {
        g_estAltitude.est.aglAltitude = g_estAltitude.sonar.meanAltitude;
        g_estAltitude.est.baroGroundOffset = g_estAltitude.baro.meanAltitude - g_estAltitude.est.aglAltitude;
    }
    else {
        g_estAltitude.est.aglAltitude = 0;
        g_estAltitude.est.baroGroundOffset = g_estAltitude.baro.meanAltitude;
    }

    // Execute callbacks to ALTHOLD and SNL
    signalAltitudeResetConditionToALTHOLD();
    signalAltitudeResetConditionToSNL();
}

static void processEstimatorStateTransition(uint32_t currentTime)
{
    estAltitudeQuality_e newQuality = g_estAltitude.est.quality;
    bool resetEstAglAlt = false;

    switch (g_estAltitude.est.quality) {
        case EST_ALT_QUAL_DR:   // BARO
            if (g_estAltitude.sonar.sonarConsistency >= SONAR_CONSISTENCY_HIGH_THRESHOLD && allowSonarForAglCorrection()) {
                // High confidence (DR -> HIGH)
                newQuality = EST_ALT_QUAL_HIGH;
                resetEstAglAlt = true;
            }
            else if (g_estAltitude.sonar.sonarConsistency >= SONAR_CONSISTENCY_LOW_THRESHOLD && allowSonarForAglCorrection()) {
                // Mid confidence (stay at DR)
                newQuality = EST_ALT_QUAL_DR;
            }
            else {
                // Low confidence (stay at DR)
                newQuality = EST_ALT_QUAL_DR;
            }
            break;

        case EST_ALT_QUAL_MID:  // BARO + SONAR
            if (g_estAltitude.sonar.sonarConsistency >= SONAR_CONSISTENCY_HIGH_THRESHOLD && allowSonarForAglCorrection()) {
                // High confidence (MID -> HIGH)
                newQuality = EST_ALT_QUAL_HIGH;
            }
            else if (g_estAltitude.sonar.sonarConsistency >= SONAR_CONSISTENCY_LOW_THRESHOLD && allowSonarForAglCorrection()) {
                // Mid confidence (stay at MID)
                newQuality = EST_ALT_QUAL_MID;
            }
            else {
                // Low confidence (MID -> DR)
                newQuality = EST_ALT_QUAL_DR;
            }
            break;

        case EST_ALT_QUAL_HIGH: // SONAR only
            if (g_estAltitude.sonar.sonarConsistency >= SONAR_CONSISTENCY_HIGH_THRESHOLD && allowSonarForAglCorrection()) {
                // High confidence (stay at HIGH)
                newQuality = EST_ALT_QUAL_HIGH;
            }
            else if (g_estAltitude.sonar.sonarConsistency >= SONAR_CONSISTENCY_LOW_THRESHOLD && allowSonarForAglCorrection()) {
                // Mid confidence (HIGH -> MID)
                newQuality = EST_ALT_QUAL_MID;
            }
            else {
                // Low confidence (HIGH -> DR)
                newQuality = EST_ALT_QUAL_DR;
            }
            break;
    }

    // Reset the estimator if needed
    if (resetEstAglAlt) {
        resetEstimatedAltitudeToSonar();
    }

    // Change the quality
    if (g_estAltitude.est.quality != newQuality) {
        g_estAltitude.est.quality = newQuality;
        g_estAltitude.est.qualityChangeTime = currentTime;
    }
}

void calculateEstimatedAltitude(uint32_t currentTime)
{
    float altInnov;
    bool areSensorsReady = isImuReady() && sensors(SENSOR_BARO) && isBaroCalibrationComplete();

    /* Update topics */
    updateIMUReadings(currentTime);
    updateBaroReadings(currentTime);
    updateSonarReadings(currentTime);

    /* If IMU is not ready or no BARO - can't really do anything */
    /* If we are onground and armed, reset altEst, also when we are not armed */
    if ((!areSensorsReady) || (ARMING_FLAG(ARMED) && (g_onGroundState == ONGROUND)) || !ARMING_FLAG(ARMED)) {
        resetEstimatedAltitude();

        if (!areSensorsReady) {
            return;
        }
    }

    /* Process estimator state transitions */
    processEstimatorStateTransition(currentTime);
#ifdef DEBUG_AGL_RESET
    debug[0] = g_estAltitude.est.quality;
    debug[1] = g_estAltitude.sonar.sonarConsistency * 100;
    debug[2] = g_estAltitude.est.aglAltitude;
    debug[3] = g_estAltitude.est.aglVelocity;
#endif

    /* Update estimate from ACCEL */
    g_estAltitude.est.aglAltitude += g_estAltitude.est.aglVelocity * g_estAltitude.imu.deltaTime
                                  + g_estAltitude.imu.correctedAccZ * g_estAltitude.imu.deltaTime * g_estAltitude.imu.deltaTime / 2.0f;
    g_estAltitude.est.aglVelocity += g_estAltitude.imu.correctedAccZ * g_estAltitude.imu.deltaTime;

    if (crashResetEstimatedAltitude()) {
        // Zero out estimated velocity, reset AGL to average known sonar, reset baro offset
        resetEstimatedAltitudeToSonar();
        g_estAltitude.est.aglVelocity = 0;

        // Boost sonar/baro weights for a short time to recover from the reset faster
        g_estAltitude.est.weightBoost = 1.0f;

        // Consume all sensors and skip the correction phase
        g_estAltitude.sonar.isUpdated = false;
        g_estAltitude.baro.isUpdated = false;
        return;
    }
    else {
        g_estAltitude.est.weightBoost *= WEIGHT_BOOST_ALPHA;
    }

#ifdef DEBUG_AGL_CRASH_RESET
    debug[0] = 100.0f * g_estAltitude.est.weightBoost;
#endif

    /* Process sonar updates */
    if (sensors(SENSOR_SONAR)) {
        if (g_estAltitude.est.quality >= EST_ALT_QUAL_MID) {
            float sonarWeightScaleFactor = scaleRangef(g_estAltitude.est.weightBoost, 0.0f, 1.0f, 1.0f, WEIGHT_BOOST_SONAR);

            /* If sonar is updated we might want to consume the measurement for baro offset correction (if quality is HIGH) */
            if (g_estAltitude.sonar.isUpdated && g_estAltitude.est.quality == EST_ALT_QUAL_HIGH) {
                /* Calculate ground offset for baro */
                if (g_estAltitude.sonar.sonarOutlierCount == 0) { // only recalculate baro bias when we have no outliers in our measurements
                    /* Sonar is consistent and timely updated - correct baro offset in a slow and smooth manner */
                    g_estAltitude.est.baroGroundOffset = g_estAltitude.baro.meanAltitude - g_estAltitude.sonar.meanAltitude;
                }
            } // g_estAltitude.sonar.isUpdated

            /* Act according on number of consecutive outliers:
             *      if zero - use current sonarWeightScaleFactor
             *      if one  - skip the measurement completely
             *      if >1   - reduce weight of this measurement
             *
             * In normal operation sonarWeightScaleFactor is an upscaler to speed up recovery from a ground hit
             */
            if (g_estAltitude.sonar.sonarOutlierCount == 1 && g_estAltitude.sonar.isOutlier) {
                sonarWeightScaleFactor = 0.0f;
            }

            /* Use latest sonar altitude as an attractor for estimate (correct always, not necessarily when sonar is updated) */
            altInnov = g_estAltitude.est.aglAltitude - g_estAltitude.sonar.currentAltitude;

            const float bellCurveScaler = 0.1f + 0.9f * expf(-sq(altInnov) / (2.0f * sq(SONAR_ACCEPTANCE_RANGE_CM)));
            const float totalSonarWeightScaler = bellCurveScaler *
                                     g_estAltitude.sonar.sonarConsistency *
                                     sonarWeightScaleFactor;

#ifdef DEBUG_AGL_OUTLIER_FILTER
            debug[0] = g_estAltitude.sonar.currentAltitude;
            debug[1] = g_estAltitude.est.aglAltitude;
            debug[2] = g_estAltitude.sonar.sonarOutlierCount;
            debug[3] = 100 * totalSonarWeightScaler;
#endif

            g_estAltitude.est.aglAltitude -= altInnov * altHoldSettings->alt_sonar_z_w * totalSonarWeightScaler * g_estAltitude.imu.deltaTime;
            g_estAltitude.est.aglVelocity -= altInnov * altHoldSettings->alt_sonar_v_w * sq(totalSonarWeightScaler) * g_estAltitude.imu.deltaTime;
        }

        /* Reset the updated flag - even if didn't use the measurement - we don't want to keep it */
        g_estAltitude.sonar.isUpdated = false;
    } // SONAR

    /* Baro is assumed to be present and operational */
    if (sensors(SENSOR_BARO) && (g_estAltitude.est.quality == EST_ALT_QUAL_MID || g_estAltitude.est.quality == EST_ALT_QUAL_DR)) {
        const float baroWeightBoost = scaleRangef(g_estAltitude.est.weightBoost, 0.0f, 1.0f, 1.0f, WEIGHT_BOOST_BARO);
        const float totalBaroWeightScaler = baroWeightBoost * (1.0f - g_estAltitude.sonar.sonarConsistency);

        /* Apply baro corrections */
        altInnov = g_estAltitude.est.aglAltitude - (g_estAltitude.baro.currentAltitude - g_estAltitude.est.baroGroundOffset);
        g_estAltitude.est.aglAltitude -= altInnov * altHoldSettings->alt_baro_z_w * totalBaroWeightScaler * g_estAltitude.imu.deltaTime;
        g_estAltitude.est.aglVelocity -= altInnov * altHoldSettings->alt_baro_v_w * sq(totalBaroWeightScaler) * g_estAltitude.imu.deltaTime;

        /* Update accZ bias from baro - this is reliable enough if weight is small */
        if (g_estAltitude.baro.isUpdated) {
            if (ARMING_FLAG(ARMED)) {
                g_estAltitude.imu.biasAccZ -= altInnov * altHoldSettings->alt_baro_accz_bias_w * g_estAltitude.baro.deltaTime;
            }

            /* Reset the updated flag - measurement was consumed by estimation and no longer usable */
            g_estAltitude.baro.isUpdated = false;
        }
    }

    /* Update crash detection */
    //setZVel(g_estAltitude.est.aglVelocity);

    /* Allow sonar next cycle (unless something will disallow in again) */
//    sonarAllowanceFlag = SONAR_ALLOWED;

    g_altitudeHoldLog.altHoldAccZ = g_estAltitude.imu.correctedAccZ;
    g_altitudeHoldLog.logEstAglAlt = g_estAltitude.est.aglAltitude;
    g_altitudeHoldLog.logEstAglVel = g_estAltitude.est.aglVelocity;
    g_altitudeHoldLog.sonarConsistency = g_estAltitude.sonar.sonarConsistency * 100;
    g_altitudeHoldLog.estAltQuality = g_estAltitude.est.quality;
    g_altitudeHoldLog.estBaroOffset = g_estAltitude.est.baroGroundOffset;
}

