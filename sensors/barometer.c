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

#include "common/maths.h"

#include "drivers/barometer.h"
#include "drivers/system.h"
#include "config/config.h"

#include "sensors/barometer.h"

baro_t baro;                        // barometer access functions
uint16_t calibratingB = 0;      // baro calibration = get new ground pressure value
int32_t baroPressure = 0;
int32_t baroTemperature = 0;
int32_t BaroAlt = 0;

#ifdef BARO

static int32_t baroGroundAltitude = 0;
static int32_t baroGroundPressure = 0;

static barometerConfig_t *barometerConfig;

void useBarometerConfig(barometerConfig_t *barometerConfigToUse)
{
    barometerConfig = barometerConfigToUse;
}

bool isBaroCalibrationComplete(void)
{
    return calibratingB == 0;
}

void baroSetCalibrationCycles(uint16_t calibrationCyclesRequired)
{
    calibratingB = calibrationCyclesRequired;
}

static bool baroReady = false;

#define PRESSURE_SAMPLES_MEDIAN 3

static int32_t applyBarometerMedianFilter(int32_t newPressureReading)
{
    static int32_t barometerFilterSamples[PRESSURE_SAMPLES_MEDIAN];
    static int currentFilterSampleIndex = 0;
    static bool medianFilterReady = false;
    int nextSampleIndex;
    
    nextSampleIndex = (currentFilterSampleIndex + 1);
    if (nextSampleIndex == PRESSURE_SAMPLES_MEDIAN) {
        nextSampleIndex = 0;
        medianFilterReady = true;
    }

    barometerFilterSamples[currentFilterSampleIndex] = newPressureReading;
    currentFilterSampleIndex = nextSampleIndex;
    
    if (medianFilterReady)
        return quickMedianFilter3(barometerFilterSamples);
    else
        return newPressureReading;
}

typedef enum {
    BAROMETER_NEEDS_SAMPLES = 0,
    BAROMETER_NEEDS_CALCULATION,
    BAROMETER_NEEDS_PROCESSING
} barometerState_e;

bool isBaroReady(void) {
	return baroReady;
}

void baroUpdate(uint32_t currentTime)
{
    static uint32_t baroDeadline = 0;
    static barometerState_e state = BAROMETER_NEEDS_SAMPLES;

    if ((int32_t)(currentTime - baroDeadline) < 0)
        return;

    baroDeadline = 0;
    switch (state) {
        case BAROMETER_NEEDS_SAMPLES:
            baro.get_ut();
            baro.start_up();
            state = BAROMETER_NEEDS_CALCULATION;
            baroDeadline += baro.up_delay;
        break;

        case BAROMETER_NEEDS_CALCULATION:
            baro.get_up();
            baro.start_ut();
            baroDeadline += baro.ut_delay;
            baro.calculate(&baroPressure, &baroTemperature);
            state = BAROMETER_NEEDS_PROCESSING;
        break;

        case BAROMETER_NEEDS_PROCESSING:
            state = BAROMETER_NEEDS_SAMPLES;
            if (barometerConfig->apply_median_filter) {
                baroPressure = applyBarometerMedianFilter(baroPressure);
            }
        break;
    }
    baroDeadline += micros();        // make sure deadline is set after calling baro callbacks
}

static void performBaroCalibrationCycle(void)
{
    baroGroundPressure -= baroGroundPressure / 8;
    baroGroundPressure += baroPressure;
    baroGroundAltitude = (1.0f - powf((baroGroundPressure / 8) / 101325.0f, 0.190295f)) * 4433000.0f;

    calibratingB--;
}

int32_t baroCalculateAltitude(void)
{
    // AMIMON: Moved barometer calibration to avoid separate call to performBaroCalibrationCycle()
    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        BaroAlt = 0;
    }
    else {
        // calculates height from ground via baro readings
        // see: https://github.com/diydrones/ardupilot/blob/master/libraries/AP_Baro/AP_Baro.cpp#L140
        BaroAlt = lrintf((1.0f - powf((float)(baroPressure) / 101325.0f, 0.190295f)) * 4433000.0f); // in cm
        BaroAlt -= baroGroundAltitude;
    }

    return BaroAlt;
}

#endif /* BARO */
