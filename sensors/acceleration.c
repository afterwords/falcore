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

#include "platform.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "io/beeper.h"
#include "sensors/boardalignment.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/acceleration.h"

int16_t accADCRaw[XYZ_AXIS_COUNT];
int32_t accADC[XYZ_AXIS_COUNT];

acc_t acc;                       // acc access functions
sensor_align_e accAlign = 0;
uint16_t acc_1G = 256;          // this is the 1G measured acceleration.

uint16_t calibratingA = 0;      // the calibration is done is the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.

static flightDynamicsTrims_t * accZero;
static flightDynamicsTrims_t * accGain;

static int8_t accLpfCutHz = 0;
static biquad_t accFilterState[XYZ_AXIS_COUNT];
static bool accFilterInitialised = false;

void accSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingA = calibrationCyclesRequired;
}

bool isAccelerationCalibrationComplete(void)
{
    return calibratingA == 0;
}

bool isOnFinalAccelerationCalibrationCycle(void)
{
    return calibratingA == 1;
}

bool isOnFirstAccelerationCalibrationCycle(void)
{
    return calibratingA == CALIBRATING_ACC_CYCLES;
}

static sensorCalibrationState_t calState;
static bool axisDataAvailable[6];
static int32_t axisAccSamples[6][3];
static int axisDataAvailableCount = 0;

bool isAccelerometerCalibrationValid(void)
{
    return (accZero->raw[X] != 0) ||
           (accZero->raw[Y] != 0) ||
           (accZero->raw[Z] != 0) ||
           (accGain->raw[X] != 4096) ||
           (accGain->raw[Y] != 4096) ||
           (accGain->raw[Z] != 4096);
}

int getAccelerometerCalibrationAxisStatus(int axis)
{
    if (isAccelerationCalibrationComplete()) {
        if (isAccelerometerCalibrationValid()) {
            return 1;
        }
        else {
            return axisDataAvailable[axis];
        }
    }
    else {
        return axisDataAvailable[axis];
    }
}

int getAxisAndDirectionIndex(int32_t sample[3])
{
    // Tolerate up to atan(1 / 1.5) = 33 deg tilt (in worst case 66 deg separation between points)
    if ((ABS(sample[Z]) / 1.5f) > ABS(sample[X]) && (ABS(sample[Z]) / 1.5f) > ABS(sample[Y])) {
        //Z-axis
        return (sample[Z] > 0) ? 0 : 1;
    }
    else if ((ABS(sample[X]) / 1.5f) > ABS(sample[Y]) && (ABS(sample[X]) / 1.5f) > ABS(sample[Z])) {
        //X-axis
        return (sample[X] > 0) ? 2 : 3;
    }
    else if ((ABS(sample[Y]) / 1.5f) > ABS(sample[X]) && (ABS(sample[Y]) / 1.5f) > ABS(sample[Z])) {
        //Y-axis
        return (sample[Y] > 0) ? 4 : 5;
    }
    else
        return -1;
}

void performAcclerationCalibration(void)
{
    int axisIndex = getAxisAndDirectionIndex(accADC);
    uint8_t axis;

    // Check if sample is usable
    if (axisIndex < 0) {
        return;
    }

    // Top-up and first calibration cycle, reset everything
    if (axisIndex == 0 && isOnFirstAccelerationCalibrationCycle()) {
        for (axis = 0; axis < 3; axis++) {
            accZero->raw[axis] = 0;
            accGain->raw[axis] = 4096;
        }

        for (axis = 0; axis < 6; axis++) {
            axisDataAvailable[axis] = false;
            axisAccSamples[axis][X] = 0;
            axisAccSamples[axis][Y] = 0;
            axisAccSamples[axis][Z] = 0;
        }

        axisDataAvailableCount = 0;
        sensorCalibrationResetState(&calState);
    }

    if (!axisDataAvailable[axisIndex]) {
        sensorCalibrationPushSampleForOffsetCalculation(&calState, accADC);
        axisAccSamples[axisIndex][X] += accADC[X];
        axisAccSamples[axisIndex][Y] += accADC[Y];
        axisAccSamples[axisIndex][Z] += accADC[Z];

        if (isOnFinalAccelerationCalibrationCycle()) {
            axisDataAvailable[axisIndex] = true;
            axisDataAvailableCount++;
        }
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        if (axisDataAvailableCount == 6) {
            float accTmp[3];
            int32_t accSample[3];

            /* Calculate offset */
            sensorCalibrationSolveForOffset(&calState, accTmp);

            for (axis = 0; axis < 3; axis++) {
                accZero->raw[axis] = lrintf(accTmp[axis]);
            }

            /* Not we can offset our accumulated averages samples and calculate scale factors and calculate gains */
            sensorCalibrationResetState(&calState);

            for (axis = 0; axis < 6; axis++) {
                accSample[X] = axisAccSamples[axis][X] / CALIBRATING_ACC_CYCLES - accZero->raw[X];
                accSample[Y] = axisAccSamples[axis][Y] / CALIBRATING_ACC_CYCLES - accZero->raw[Y];
                accSample[Z] = axisAccSamples[axis][Z] / CALIBRATING_ACC_CYCLES - accZero->raw[Z];

                sensorCalibrationPushSampleForScaleCalculation(&calState, axis / 2, accSample, acc_1G);
            }

            sensorCalibrationSolveForScale(&calState, accTmp);

            for (axis = 0; axis < 3; axis++) {
                accGain->raw[axis] = accTmp[axis] * 4096;
            }

            // Silently write EEPROM
            writeEEPROM();

            // Beepout confirmation beep
            beeper(BEEPER_ACC_CALIBRATION_COMPLETE);
        }
        else {
            beeperConfirmationBeeps(2);
        }
    }

    calibratingA--;
}

void applyAccelerationZero(flightDynamicsTrims_t * accZero, flightDynamicsTrims_t * accGain)
{
    accADC[X] = (accADC[X] - accZero->raw[X]) * accGain->raw[X] / 4096;
    accADC[Y] = (accADC[Y] - accZero->raw[Y]) * accGain->raw[Y] / 4096;
    accADC[Z] = (accADC[Z] - accZero->raw[Z]) * accGain->raw[Z] / 4096;
}

void updateAccelerationReadings(void)
{
    int8_t axis;

    if (!acc.read(accADCRaw)) {
        return;
    }

    for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) accADC[axis] = accADCRaw[axis];

    if (accLpfCutHz) {
        if (!accFilterInitialised) {
            if (targetLooptime) {  /* Initialisation needs to happen once sample rate is known */
                for (axis = 0; axis < 3; axis++) {
                    filterInitBiQuad(accLpfCutHz, &accFilterState[axis], 0);
                }

                accFilterInitialised = true;
            }
        }

        if (accFilterInitialised) {
            for (axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
                accADC[axis] = lrintf(filterApplyBiQuad((float) accADC[axis], &accFilterState[axis]));
            }
        }
    }

    if (!isAccelerationCalibrationComplete()) {
        performAcclerationCalibration();
    }

    applyAccelerationZero(accZero, accGain);

    alignSensors(accADC, accADC, accAlign);
}

void setAccelerationZero(flightDynamicsTrims_t * accZeroToUse)
{
    accZero = accZeroToUse;
}

void setAccelerationGain(flightDynamicsTrims_t * accGainToUse)
{
    accGain = accGainToUse;
}

void setAccelerationFilter(int8_t initialAccLpfCutHz)
{
    accLpfCutHz = initialAccLpfCutHz;
}
