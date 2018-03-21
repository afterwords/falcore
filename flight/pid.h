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

#define YAW_P_LIMIT_MIN 100                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_MAX 500                 // Maximum value for yaw P limiter
#define YAW_P_LIMIT_DEFAULT 160

typedef enum {
    PIDROLL,
    PIDPITCH,
    PIDYAW,
    PIDALT,
    PIDPOS,
    PIDPOSR,
    PIDNAVR,
    PIDLEVEL,
    PIDMAG,
    PIDVEL,
    PID_ITEM_COUNT
} pidIndex_e;

#define PID_RATE_P_MULTIPLIER    40.0f
#define PID_RATE_I_MULTIPLIER    4.0f
#define PID_RATE_D_MULTIPLIER    3000.0f
#define PID_LEVEL_P_MULTIPLIER   10.0f

typedef struct pidProfile_s {
    uint8_t P8[PID_ITEM_COUNT];
    uint8_t I8[PID_ITEM_COUNT];
    uint8_t D8[PID_ITEM_COUNT];

    uint8_t dterm_lpf_hz;                   // (default 17Hz, Range 1-50Hz) Used for PT1 element in PID1, PID2 and PID5
    uint8_t gyro_soft_lpf_hz;               // Gyro FIR filtering
    uint8_t acc_soft_lpf_hz;                // Set the Low Pass Filter factor for ACC. Reducing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time. Zero = no filter

    int16_t max_angle_inclination[2][2];   // max inclination allowed in angle mode (pitch/roll and +/-)

    uint16_t rollPitchItermIgnoreRate;      // Experimental threshold for ignoring iterm for pitch and roll on certain rates
    uint16_t yawItermIgnoreRate;            // Experimental threshold for ignoring iterm for yaw on certain rates

#ifdef GTUNE
    uint8_t  gtune_lolimP[3];               // [0..200] Lower limit of P during G tune
    uint8_t  gtune_hilimP[3];               // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
    uint8_t  gtune_pwr;                     // [0..10] Strength of adjustment
    uint16_t gtune_settle_time;             // [200..1000] Settle time in ms
    uint8_t  gtune_average_cycles;          // [8..128] Number of looptime cycles used for gyro average calculation
#endif
} pidProfile_t;

extern int16_t axisPID[XYZ_AXIS_COUNT];
extern int32_t axisPID_P[3], axisPID_I[3], axisPID_D[3];

void pidResetErrorGyro(void);

float rcCommandToAngleDecidegrees(pidProfile_t *pidProfile, uint8_t axis, int32_t rcCommand);
int32_t angleDecidegreesToRcCommand(pidProfile_t *pidProfile, uint8_t axis, float angle);

float rcCommandToAxisRate(uint8_t rate, int32_t rcCommand);
int32_t axisRateToRcCommand(uint8_t rate, float axisRate);

void pidResetCustomInclinationLimits(void);
void pidResetCustomAxisInclinationLimits(uint8_t axis);
void pidSetCustomAxisInclinationLimits(uint8_t axis, int16_t limitPositive, int16_t limitNegative);
