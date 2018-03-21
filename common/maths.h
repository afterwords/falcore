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

#pragma once

#ifndef sq
#define sq(x) ((x)*(x))
#endif

// Undefine this for use libc sinf/cosf. Keep this defined to use fast sin/cos approximations
#define FAST_MATH             // order 9 approximation
//#define VERY_FAST_MATH      // order 7 approximation

// Use floating point M_PI instead explicitly.
#define M_PIf       3.14159265358979323846f
#define M_LN2f      0.69314718055994530942f

#define RAD    (M_PIf / 180.0f)

#define DEGREES_TO_CENTIDEGREES(angle) ((angle) * 100)
#define CENTIDEGREES_TO_DEGREES(angle) ((angle) / 100)

#define CENTIDEGREES_TO_DECIDEGREES(angle) ((angle) / 10)
#define DECIDEGREES_TO_CENTIDEGREES(angle) ((angle) * 10)

#define DEGREES_TO_DECIDEGREES(angle) ((angle) * 10)
#define DECIDEGREES_TO_DEGREES(angle) ((angle) / 10)

#define DEGREES_TO_RADIANS(angle) ((angle) * RAD)
#define RADIANS_TO_DEGREES(angle) ((angle) / RAD)
#define DECIDEGREES_TO_RADIANS(angle) (((angle) / 10.0f) * RAD)
#define RADIANS_TO_DECIDEGREES(angle) (((angle) * 10.0f) / RAD)

#define RADIANS_TO_CENTIDEGREES(angle) (((angle) * 100.0f) / RAD)
#define CENTIDEGREES_TO_RADIANS(angle) (((angle) / 100.0f) * RAD)

#define CM_TO_METER(x) ((x) / 100.f)
#define METER_TO_CM(x) ((x) * 100.f)

#define CMS_TO_MS(x) ((x) / 100.f)
#define MS_TO_CMS(x) ((x) * 100.f)

#define GRAM_TO_KG(x) ((x) / 1000.f)
#define KG_TO_GRAM(x) ((x) * 1000.f)

#define US2S(us)    ((float)(us) * 1e-6f)
#define S2US(s)     ((float)(s) * 1e6f)
#define MS2US(ms)   ((float)(ms) * 1e3f)
#define HZ2US(hz)   (1e6f / (float)(hz))

// copied from https://code.google.com/p/cxutil/source/browse/include/cxutil/utility.h#70
#define _CHOOSE2(binoper, lexpr, lvar, rexpr, rvar)         \
    ( __extension__ ({                                      \
            __typeof__(lexpr) lvar = (lexpr);               \
            __typeof__(rexpr) rvar = (rexpr);               \
            lvar binoper rvar ? lvar : rvar;                \
        }))
#define _CHOOSE_VAR2(prefix, unique) prefix##unique
#define _CHOOSE_VAR(prefix, unique) _CHOOSE_VAR2(prefix, unique)
#define _CHOOSE(binoper, lexpr, rexpr)                   \
    _CHOOSE2(                                            \
        binoper,                                         \
        lexpr, _CHOOSE_VAR(_left, __COUNTER__),          \
        rexpr, _CHOOSE_VAR(_right, __COUNTER__)          \
        )
#define MIN(a, b) _CHOOSE(<, a, b)
#define MAX(a, b) _CHOOSE(>, a, b)

#define _ABS_II(x, var)                           \
    ( __extension__ ({                            \
        __typeof__(x) var = (x);                  \
        var < 0 ? -var : var;                     \
    }))
#define _ABS_I(x, var) _ABS_II(x, var)
#define ABS(x) _ABS_I(x, _CHOOSE_VAR(_abs, __COUNTER__))
#define SIGN(x) ((x) > 0 ? 1 : -1)

typedef struct stdev_s
{
    float m_oldM, m_newM, m_oldS, m_newS;
    int m_n;
} stdev_t;

// Floating point 3 vector.
typedef struct fp_vector {
    float X;
    float Y;
    float Z;
} t_fp_vector_def;

typedef union {
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

// Floating point Euler angles.
// Be carefull, could be either of degrees or radians.
typedef struct fp_angles {
    float roll;
    float pitch;
    float yaw;
} fp_angles_def;

typedef union {
    float raw[3];
    fp_angles_def angles;
} fp_angles_t;

typedef struct filterWithBufferSample_s {
    float value;
    uint32_t timestamp;
} filterWithBufferSample_t;

typedef struct filterWithBufferState_s {
    uint16_t filter_size;
    uint16_t sample_index;
    filterWithBufferSample_t * samples;
} filterWithBufferState_t;

typedef struct {
    float XtY[4];
    float XtX[4][4];
} sensorCalibrationState_t;

void sensorCalibrationResetState(sensorCalibrationState_t * state);
void sensorCalibrationPushSampleForOffsetCalculation(sensorCalibrationState_t * state, int32_t sample[3]);
void sensorCalibrationPushSampleForScaleCalculation(sensorCalibrationState_t * state, int axis, int32_t sample[3], int target);
void sensorCalibrationSolveForOffset(sensorCalibrationState_t * state, float result[3]);
void sensorCalibrationSolveForScale(sensorCalibrationState_t * state, float result[3]);

int32_t applyDeadband(int32_t value, int32_t deadband);

int constrain(int amt, int low, int high);
float constrainf(float amt, float low, float high);

void devClear(stdev_t *dev);
void devPush(stdev_t *dev, float x);
float devVariance(stdev_t *dev);
float devStandardDeviation(stdev_t *dev);
float degreesToRadians(int16_t degrees);

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax);
int scaleRangef(float x, float srcMin, float srcMax, float destMin, float destMax);

void normalizeV(struct fp_vector *src, struct fp_vector *dest);

void buildRotationMatrix(fp_angles_t *delta, float matrix[3][3]);
void rotateV(struct fp_vector *v, fp_angles_t *delta);

void rotVector2D(float Vector[2], int16_t rot_angle, float Result[2]);
#define rot2D_B2E(v_B, yaw_angle, v_E)  rotVector2D(v_B, yaw_angle, v_E)
#define rot2D_E2B(v_E, yaw_angle, v_B)  rotVector2D(v_E, -(yaw_angle), v_B)



int32_t quickMedianFilter3(int32_t * v);
int32_t quickMedianFilter5(int32_t * v);
int32_t quickMedianFilter7(int32_t * v);
int32_t quickMedianFilter9(int32_t * v);

#if defined(FAST_MATH) || defined(VERY_FAST_MATH)
float sin_approx(float x);
float cos_approx(float x);
float atan2_approx(float y, float x);
float acos_approx(float x);
#define tan_approx(x)       (sin_approx(x) / cos_approx(x))
#else
#define sin_approx(x)       sinf(x)
#define cos_approx(x)       cosf(x)
#define atan2_approx(y,x)   atan2f(y,x)
#define acos_approx(x)      acosf(x)
#define tan_approx(x)       tanf(x)
#endif

void arraySubInt32(int32_t *dest, int32_t *array1, int32_t *array2, int count);
uint32_t hashJOAAT(const uint8_t * data, int size);