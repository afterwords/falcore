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

#include "platform.h"
#include "build_config.h"

#include "common/maths.h"
#include "common/axis.h"

#include "drivers/sonar_hcsr04.h"
#include "drivers/gpio.h"
#include "config/runtime_config.h"
#include "config/config.h"

#include "sensors/sensors.h"
#include "sensors/battery.h"
#include "sensors/sonar.h"

// Sonar measurements are in cm, a value of SONAR_OUT_OF_RANGE indicates sonar is not in range.
// Inclination is adjusted by imu

#ifdef SONAR
int16_t sonarMaxRangeCm;
int16_t sonarMaxAltWithTiltCm;
int16_t sonarCfAltCm; // Complimentary Filter altitude
STATIC_UNIT_TESTED int16_t sonarMaxTiltDeciDegrees;

static int32_t calculatedAltitude;

const sonarHardware_t *sonarGetHardwareConfiguration(batteryConfig_t *batteryConfig)
{
#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R)
    static const sonarHardware_t sonarPWM56 = {
        .trigger_gpio = GPIOB,
        .echo_gpio = GPIOB,
        .trigger_pin = Pin_8,   // PWM5 (PB8) - 5v tolerant
        .echo_pin = Pin_9,      // PWM6 (PB9) - 5v tolerant
        .exti_line = EXTI_Line9,
        .exti_pin_source = GPIO_PinSource9,
        .exti_irqn = EXTI9_5_IRQn
    };
    static const sonarHardware_t sonarRC78 = {
        .trigger_gpio = GPIOB,
        .echo_gpio = GPIOB,
        .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_pin = Pin_1,      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    // If we are using parallel PWM for our receiver or ADC current sensor, then use motor pins 5 and 6 for sonar, otherwise use rc pins 7 and 8
    if (feature(FEATURE_RX_PARALLEL_PWM ) || (feature(FEATURE_CURRENT_METER) && batteryConfig->currentMeterType == CURRENT_SENSOR_ADC) ) {
        return &sonarPWM56;
    } else {
        return &sonarRC78;
    }
#elif defined(OLIMEXINO)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_gpio = GPIOB,
        .echo_gpio = GPIOB,
        .trigger_pin = Pin_0,   // RX7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_pin = Pin_1,      // RX8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line1,
        .exti_pin_source = GPIO_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    return &sonarHardware;
#elif defined(CC3D)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_gpio = GPIOB,
        .echo_gpio = GPIOB,
        .trigger_pin = Pin_5,   // (PB5)
        .echo_pin = Pin_0,      // (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line0,
        .exti_pin_source = GPIO_PinSource0,
        .exti_irqn = EXTI0_IRQn
    };
    return &sonarHardware;
#elif defined(SPRACINGF3)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_gpio = GPIOB,
        .echo_gpio = GPIOB,
        .trigger_pin = Pin_0,   // RC_CH7 (PB0) - only 3.3v ( add a 1K Ohms resistor )
        .echo_pin = Pin_1,      // RC_CH8 (PB1) - only 3.3v ( add a 1K Ohms resistor )
        .exti_line = EXTI_Line1,
        .exti_pin_source = EXTI_PinSource1,
        .exti_irqn = EXTI1_IRQn
    };
    return &sonarHardware;
#elif defined(CONNEXFC)
    UNUSED(batteryConfig);
    static const sonarHardware_t const sonarHardware = {
        .trigger_gpio = GPIOA,
        .echo_gpio = GPIOA,
        .trigger_pin = Pin_7,   // PA7
        .echo_pin = Pin_2,      // PA2
        .exti_line = EXTI_Line2,
        .exti_pin_source = GPIO_PinSource2,
        .exti_irqn = EXTI2_TS_IRQn
    };
    return &sonarHardware;
#elif defined(UNIT_TEST)
    UNUSED(batteryConfig);
    return 0;
#else
#error Sonar not defined for target
#endif
}

// (PI/1800)^2/2, coefficient of x^2 in Taylor expansion of cos(x)
#define coefX2 1.52309E-06f
#define cosDeciDegrees(x) (1.0f - x * x * coefX2)


void sonarInit(const sonarHardware_t *sonarHardware)
{
    sonarRange_t sonarRange;

    hcsr04_init(sonarHardware, &sonarRange);
    sensorsSet(SENSOR_SONAR);
    sonarMaxRangeCm = sonarRange.maxRangeCm;
    sonarCfAltCm = sonarMaxRangeCm / 2;
    sonarMaxTiltDeciDegrees =  sonarRange.detectionConeExtendedDeciDegrees / 2;
    sonarMaxAltWithTiltCm = sonarMaxRangeCm * cosDeciDegrees(sonarMaxTiltDeciDegrees);
    calculatedAltitude = SONAR_OUT_OF_RANGE;
}

void sonarUpdate(void)
{
    hcsr04_start_reading();
}

/* not used currently
static int32_t applyMedianFilter(int32_t newReading)
{
    #define DISTANCE_SAMPLES_MEDIAN 5
    static int32_t filterSamples[DISTANCE_SAMPLES_MEDIAN];
    static int filterSampleIndex = 0;
    static bool medianFilterReady = false;

    if (newReading > SONAR_OUT_OF_RANGE) {// only accept samples that are in range
        filterSamples[filterSampleIndex] = newReading;
        ++filterSampleIndex;
        if (filterSampleIndex == DISTANCE_SAMPLES_MEDIAN) {
            filterSampleIndex = 0;
            medianFilterReady = true;
        }
    }
    return medianFilterReady ? quickMedianFilter5(filterSamples) : newReading;
}
*/

/**
 * Get the last distance measured by the sonar in centimeters. When the ground is too far away, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarRead(void)
{
    int32_t newSonarReading = hcsr04_get_distance();

    if (newSonarReading > HCSR04_MAX_RANGE_CM)
        newSonarReading = SONAR_OUT_OF_RANGE;

    if (newSonarReading > SONAR_OUT_OF_RANGE) {
        //return applyMedianFilter(newSonarReading);
        return newSonarReading;
    }
    else {
        return newSonarReading == SONAR_OUT_OF_RANGE ? SONAR_OUT_OF_RANGE : SONAR_HW_FAILURE;
    }
}

/**
 * Detect sonar HW failure
 */
bool sonarIsAvailable(void)
{
    return sensors(SENSOR_SONAR) && (sonarRead() != SONAR_HW_FAILURE);
}

/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, SONAR_OUT_OF_RANGE is returned.
 */

int32_t sonarCalculateAltitudeWithTilt(int32_t sonarAlt, float cosTiltAngle, float cosTiltSensor)
{
    if (sonarAlt <= SONAR_OUT_OF_RANGE) {
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    }
    else {
        if (cosTiltAngle < cos_approx(DECIDEGREES_TO_RADIANS(sonarMaxTiltDeciDegrees)))
            calculatedAltitude = SONAR_OUT_OF_RANGE;
        else
            calculatedAltitude = sonarAlt * cosTiltSensor;
    }

    return calculatedAltitude;
}


/**
 * Apply tilt correction to the given raw sonar reading in order to compensate for the tilt of the craft when estimating
 * the altitude. Returns the computed altitude in centimeters.
 *
 * When the ground is too far away or the tilt is too large, SONAR_OUT_OF_RANGE is returned.
 */
int32_t sonarCalculateAltitude(int32_t sonarAlt, float cosTiltAngle)
{
    if (sonarAlt <= SONAR_OUT_OF_RANGE) {
        calculatedAltitude = SONAR_OUT_OF_RANGE;
    }
    else {
        if (cosTiltAngle < cos_approx(DECIDEGREES_TO_RADIANS(sonarMaxTiltDeciDegrees)))
            calculatedAltitude = SONAR_OUT_OF_RANGE;
        else
            calculatedAltitude = sonarAlt * cosTiltAngle;
    }

    return calculatedAltitude;
}

/**
 * Get the latest altitude that was computed by a call to sonarCalculateAltitude(), or SONAR_OUT_OF_RANGE if sonarCalculateAltitude
 * has never been called.
 */
int32_t sonarGetLatestAltitude(void)
{
    return calculatedAltitude;
}

#endif
