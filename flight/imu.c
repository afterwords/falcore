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

// Inertial Measurement Unit (IMU)

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "common/maths.h"

#include "build_config.h"
#include "platform.h"
#include "debug.h"

#include "common/filter.h"
#include "common/axis.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/crash_detection.h"

#include "io/gps.h"

#include "config/runtime_config.h"

/**
 * In Cleanflight accelerometer is aligned in the following way:
 *      X-axis = Forward
 *      Y-axis = Left
 *      Z-axis = Up
 * Our INAV uses different convention
 *      X-axis = North/Forward
 *      Y-axis = East/Right
 *      Z-axis = Up
 */

// AMIMON:
// IMU code is rewritten to use R. Mahony's DCM filter described in "Non-linear complementary filters on the special orthogonal group"
// (https://hal-unice.archives-ouvertes.fr/hal-00488376/document),
// initially implemented by S. Madgwick (http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/).

// the limit (in degrees/second) beyond which we stop integrating
// omega_I. At larger spin rates the DCM PI controller can get 'dizzy'
// which results in false gyro drift. See
// http://gentlenav.googlecode.com/files/fastRotations.pdf
#define SPIN_RATE_LIMIT         20

t_fp_vector imuAccelInEarthFrame;
t_fp_vector imuMeasuredAccelBF;
t_fp_vector imuMeasuredRotationBF;

float smallAngleCosZ = 0;
float magneticDeclination = 0.0f;       // calculated at startup from config
static imuRuntimeConfig_t *imuRuntimeConfig;
static pidProfile_t *pidProfile;
static bool gravityCalibrationFinished;

STATIC_UNIT_TESTED float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to earth frame
static float rMat[3][3];

attitudeEulerAngles_t attitude = { { 0, 0, 0 } };     // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800

static float gyroScale;

STATIC_UNIT_TESTED void imuComputeRotationMatrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuConfigure(imuRuntimeConfig_t *initialImuRuntimeConfig, pidProfile_t *initialPidProfile)
{
    imuRuntimeConfig = initialImuRuntimeConfig;
    pidProfile = initialPidProfile;
}

void imuInit(void)
{
    int axis;

    smallAngleCosZ = cos_approx(degreesToRadians(imuRuntimeConfig->small_angle));
    gyroScale = gyro.scale * (M_PIf / 180.0f);  // gyro output scaled to rad per second

    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuAccelInEarthFrame.A[axis] = 0;
    }

    imuComputeRotationMatrix();

    gravityCalibrationFinished = false;
}

void imuTransformVectorBodyToEarth(t_fp_vector * v)
{
    float x,y,z;

    /* From body frame to earth frame */
    x = rMat[0][0] * v->V.X + rMat[0][1] * v->V.Y + rMat[0][2] * v->V.Z;
    y = rMat[1][0] * v->V.X + rMat[1][1] * v->V.Y + rMat[1][2] * v->V.Z;
    z = rMat[2][0] * v->V.X + rMat[2][1] * v->V.Y + rMat[2][2] * v->V.Z;

    v->V.X = x;
    v->V.Y = -y;
    v->V.Z = z;
}

// rotate acc into Earth frame and calculate acceleration in it
void imuCalculateAccelerationInEarthFrame(void)
{
    static float estimatedGravityOffset = GRAVITY_CMSS;
    static uint32_t gravityOffsetCalibrationTimeout;

    /* Rotate vector to Earth frame - from Forward-Right-Down to North-East-Up*/
    imuAccelInEarthFrame = imuMeasuredAccelBF;
    imuTransformVectorBodyToEarth(&imuAccelInEarthFrame);

    // If we are calibrating accelerometer chip - we have to re-calibrate Gravity offset
    if (!isAccelerationCalibrationComplete()) {
        gravityCalibrationFinished = false;
    }

    if (!ARMING_FLAG(ARMED) && isGyroCalibrationComplete() && !gravityCalibrationFinished) {
        const float gravityOffsetError = imuAccelInEarthFrame.V.Z - estimatedGravityOffset;
        // Slowly converge on calibrated gravity while level
        estimatedGravityOffset += gravityOffsetError * 0.0025f;

        if (ABS(gravityOffsetError) < 0.0025f * GRAVITY_CMSS) {  // Error should be within 0.25% * G

            if ((millis() - gravityOffsetCalibrationTimeout) >= 250) {
                gravityCalibrationFinished = true;
            }
        }
        else {
            gravityOffsetCalibrationTimeout = millis();
        }
    }

    imuAccelInEarthFrame.V.Z -= estimatedGravityOffset;

#ifdef DEBUG_GRAVITY_CALIBRATION
    debug[0] = gravityCalibrationFinished;
    debug[1] = estimatedGravityOffset;
    debug[2] = gravityOffsetCalibrationTimeout;
    debug[3] = imuAccelInEarthFrame.V.Z;
#endif

#ifdef DEBUG_IMU_ACCEL
    debug[0] = acc_1G;
    debug[1] = imuAccelInEarthFrame.V.Z;
    debug[2] = estimatedGravityOffset;
#endif
}

bool isGravityCalibrationComplete(void)
{
    return gravityCalibrationFinished;
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

void imuComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
    if (initialRoll > 1800) initialRoll -= 3600;
    if (initialPitch > 1800) initialPitch -= 3600;
    if (initialYaw > 1800) initialYaw -= 3600;

    float cosRoll = cos_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);
    float sinRoll = sin_approx(DECIDEGREES_TO_RADIANS(initialRoll) * 0.5f);

    float cosPitch = cos_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);
    float sinPitch = sin_approx(DECIDEGREES_TO_RADIANS(initialPitch) * 0.5f);

    float cosYaw = cos_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);
    float sinYaw = sin_approx(DECIDEGREES_TO_RADIANS(-initialYaw) * 0.5f);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

    imuComputeRotationMatrix();
}

static bool imuUseFastGains(void)
{
    return !ARMING_FLAG(ARMED) && millis() < 20000;
}

static float imuGetPGainScaleFactor(void)
{
    if (imuUseFastGains()) {
        return 10.0f;
    }
    else {
        return 1.0f;
    }
}

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useCOG, float courseOverGround)
{
    static float integralAccX = 0.0f,  integralAccY = 0.0f, integralAccZ = 0.0f;    // integral error terms scaled by Ki
    static float integralMagX = 0.0f,  integralMagY = 0.0f, integralMagZ = 0.0f;    // integral error terms scaled by Ki
    float recipNorm;
    float ex, ey, ez;
    float qa, qb, qc;

    /* Calculate general spin rate (rad/s) */
    float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    /* Step 1: Yaw correction */
    // Use measured magnetic field vector
    if (useMag || useCOG) {
        float kpMag = imuRuntimeConfig->dcm_kp_mag * imuGetPGainScaleFactor();

        recipNorm = mx * mx + my * my + mz * mz;
        if (useMag && recipNorm > 0.01f) {
            // Normalise magnetometer measurement
            recipNorm = invSqrt(recipNorm);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
            // This way magnetic field will only affect heading and wont mess roll/pitch angles

            // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
            // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
            float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
            float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
            float bx = sqrtf(hx * hx + hy * hy);

            // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
            float ez_ef = -(hy * bx);

            // Rotate mag error vector back to BF and accumulate
            ex = rMat[2][0] * ez_ef;
            ey = rMat[2][1] * ez_ef;
            ez = rMat[2][2] * ez_ef;
        }
        else if (useCOG) {
            // Use raw heading error (from GPS or whatever else)
            while (courseOverGround >  M_PIf) courseOverGround -= (2.0f * M_PIf);
            while (courseOverGround < -M_PIf) courseOverGround += (2.0f * M_PIf);

            // William Premerlani and Paul Bizard, Direction Cosine Matrix IMU - Eqn. 22-23
            // (Rxx; Ryx) - measured (estimated) heading vector (EF)
            // (cos(COG), sin(COG)) - reference heading vector (EF)
            // error is cross product between reference heading and estimated heading (calculated in EF)
            float ez_ef = - sin_approx(courseOverGround) * rMat[0][0] - cos_approx(courseOverGround) * rMat[1][0];

            ex = rMat[2][0] * ez_ef;
            ey = rMat[2][1] * ez_ef;
            ez = rMat[2][2] * ez_ef;
        }
        else {
            ex = 0;
            ey = 0;
            ez = 0;
        }

        // Compute and apply integral feedback if enabled
        if(imuRuntimeConfig->dcm_ki_mag > 0.0f) {
            // Stop integrating if spinning beyond the certain limit
            if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
                integralMagX += imuRuntimeConfig->dcm_ki_mag * ex * dt;    // integral error scaled by Ki
                integralMagY += imuRuntimeConfig->dcm_ki_mag * ey * dt;
                integralMagZ += imuRuntimeConfig->dcm_ki_mag * ez * dt;

                gx += integralMagX;
                gy += integralMagY;
                gz += integralMagZ;
            }
        }

        // Calculate kP gain and apply proportional feedback
        gx += kpMag * ex;
        gy += kpMag * ey;
        gz += kpMag * ez;
    }


    /* Step 2: Roll and pitch correction -  use measured acceleration vector */
    if (useAcc) {
        float kpAcc = imuRuntimeConfig->dcm_kp_acc * imuGetPGainScaleFactor();

        recipNorm = ax * ax + ay * ay + az * az;
        if (recipNorm > 0.01f) {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(recipNorm);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Error is sum of cross product between estimated direction and measured direction of gravity
            ex = (ay * rMat[2][2] - az * rMat[2][1]);
            ey = (az * rMat[2][0] - ax * rMat[2][2]);
            ez = (ax * rMat[2][1] - ay * rMat[2][0]);

            // Compute and apply integral feedback if enabled
            if(imuRuntimeConfig->dcm_ki_acc > 0.0f) {
                // Stop integrating if spinning beyond the certain limit
                if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
                    integralAccX += imuRuntimeConfig->dcm_ki_acc * ex * dt;    // integral error scaled by Ki
                    integralAccY += imuRuntimeConfig->dcm_ki_acc * ey * dt;
                    integralAccZ += imuRuntimeConfig->dcm_ki_acc * ez * dt;

                    gx += integralAccX;
                    gy += integralAccY;
                    gz += integralAccZ;
                }
            }

            // Calculate kP gain and apply proportional feedback
            gx += kpAcc * ex;
            gy += kpAcc * ey;
            gz += kpAcc * ez;
        }
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();
}

STATIC_UNIT_TESTED void imuUpdateEulerAngles(void)
{
    /* Compute pitch/roll angles */
    attitude.values.roll = lrintf(atan2_approx(rMat[2][1], rMat[2][2]) * (1800.0f / M_PIf));
    attitude.values.pitch = lrintf(((0.5f * M_PIf) - acos_approx(-rMat[2][0])) * (1800.0f / M_PIf));
    attitude.values.yaw = lrintf((-atan2_approx(rMat[1][0], rMat[0][0]) * (1800.0f / M_PIf) + magneticDeclination));

    if (attitude.values.yaw < 0)
        attitude.values.yaw += 3600;

    /* Update small angle state */
    if (rMat[2][2] > smallAngleCosZ) {
        ENABLE_STATE(SMALL_ANGLE);
    } else {
        DISABLE_STATE(SMALL_ANGLE);
    }
}

static bool imuIsAccelerometerHealthy(void)
{
    int32_t axis;
    int32_t accMagnitude = 0;

    for (axis = 0; axis < 3; axis++) {
        accMagnitude += (int32_t)accADC[axis] * accADC[axis];
    }

    accMagnitude = accMagnitude * 100 / ((int32_t)acc_1G * acc_1G);

    // Accept accel readings only in range 0.90g - 1.10g
    return (81 < accMagnitude) && (accMagnitude < 121);
}

static bool isMagnetometerHealthy(void)
{
    return (magADC[X] != 0) && (magADC[Y] != 0) && (magADC[Z] != 0);
}

static void imuCalculateEstimatedAttitude(void)
{
    static uint32_t previousIMUUpdateTime = 0;
    static bool isImuInitialized = false;
    static bool gpsHeadingInitialized = false;
    float courseOverGround = 0;

    bool useAcc = false;
    bool useMag = false;
    bool useCOG = false;

    uint32_t currentTime = micros();
    uint32_t deltaT = currentTime - previousIMUUpdateTime;
    previousIMUUpdateTime = currentTime;

    if (!isImuInitialized) {
        /* Initialize initial attitude guess from accelerometer and magnetometer readings */
        attitude.values.roll = RADIANS_TO_DECIDEGREES(atan2_approx(accADC[Y], accADC[Z]));
        attitude.values.pitch = RADIANS_TO_DECIDEGREES(atan2_approx(-accADC[X], sqrtf(accADC[Y] * accADC[Y] + accADC[Z] * accADC[Z])));

        imuComputeQuaternionFromRPY(attitude.values.roll, attitude.values.pitch, 0);

#if defined(MAG)
        if (sensors(SENSOR_MAG)) {
            /* Wait until compass is read at least once */
            if (isCompassReady()) {
                t_fp_vector estM;

                estM.V.X = magADC[X];
                estM.V.Y = magADC[Y];
                estM.V.Z = magADC[Z];

                /* Pre-compute orientation quaternion and rotate measured mag field vector to earth frame */
                imuComputeQuaternionFromRPY(attitude.values.roll, attitude.values.pitch, 0);
                imuTransformVectorBodyToEarth(&estM);

                /* Calculate yaw from mag vector */
                attitude.values.yaw = RADIANS_TO_DECIDEGREES(-atan2_approx(estM.V.Y, estM.V.X));

                if (attitude.values.yaw < 0)
                    attitude.values.yaw += 3600;

                imuComputeQuaternionFromRPY(attitude.values.roll, attitude.values.pitch, attitude.values.yaw);
                isImuInitialized = true;
            }
        }
        else {
            isImuInitialized = true;
        }
#else
        isImuInitialized = true;
#endif
    }
    else {
        if (imuIsAccelerometerHealthy()) {
            useAcc = true;
        }

        if (sensors(SENSOR_MAG) && isMagnetometerHealthy()) {
            useMag = true;
        }
#if defined(GPS)
        else if (STATE(FIXED_WING) && sensors(SENSOR_GPS) && STATE(GPS_FIX) && GPS_numSat >= 5 && GPS_speed >= 300) {
            // In case of a fixed-wing aircraft we can use GPS course over ground to correct heading
            if (gpsHeadingInitialized) {
                courseOverGround = DECIDEGREES_TO_RADIANS(GPS_ground_course);
                useCOG = true;
            }
            else {
                // Re-initialize quaternion from known Roll, Pitch and GPS heading
                imuComputeQuaternionFromRPY(attitude.values.roll, attitude.values.pitch, GPS_ground_course);
                gpsHeadingInitialized = true;
            }
        }
#endif

        imuMahonyAHRSupdate(deltaT * 1e-6f, imuMeasuredRotationBF.A[X], imuMeasuredRotationBF.A[Y], imuMeasuredRotationBF.A[Z],
                            useAcc, imuMeasuredAccelBF.A[X], imuMeasuredAccelBF.A[Y], imuMeasuredAccelBF.A[Z],
                            useMag, magADC[X], magADC[Y], magADC[Z],
                            useCOG, courseOverGround);
    }

    imuUpdateEulerAngles();

    imuCalculateAccelerationInEarthFrame(); // rotate acc vector into earth frame
}

static void imuUpdateMeasuredAcceleration(void)
{
    int axis;

    /* Read accelerometer */
    updateAccelerationReadings();

    /* Convert acceleration to cm/s/s */
    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        imuMeasuredAccelBF.A[axis] = accADC[axis] * (GRAVITY_CMSS / acc_1G);
    }
}

/* Calculate rotation rate in rad/s in body frame */
static void imuUpdateMeasuredRotationRate(void)
{
    int axis;

    for (axis = 0; axis < 3; axis++) {
        imuMeasuredRotationBF.A[axis] = gyroADC[axis] * gyroScale;
    }
}

void imuUpdate(void)
{
    gyroUpdate();

    if (sensors(SENSOR_ACC)) {
        imuUpdateMeasuredAcceleration();
        imuUpdateMeasuredRotationRate();
        imuCalculateEstimatedAttitude();

       
    } else {
        accADC[X] = 0;
        accADC[Y] = 0;
        accADC[Z] = 0;
    }
}

bool isImuReady(void)
{
    return sensors(SENSOR_ACC) && isGyroCalibrationComplete() && isGravityCalibrationComplete() && isAccelerationCalibrationComplete();
}

float calculateCosTiltAngle(void)
{
    return rMat[2][2];
}

float calculateThrottleTiltCompensationFactor(uint8_t throttleTiltCompensationStrength)
{
    if (throttleTiltCompensationStrength) {
        float tiltCompFactor = 1.0f / constrainf(rMat[2][2], 0.6f, 1.0f);  // max tilt about 50 deg
        return 1.0f + (tiltCompFactor - 1.0f) * (throttleTiltCompensationStrength / 100.f);
    }
    else {
        return 1.0f;
    }
}
