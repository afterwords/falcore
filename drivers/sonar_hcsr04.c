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

#include "system.h"
#include "gpio.h"
#include "nvic.h"

#include "sonar_hcsr04.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When triggered it sends out a series of 40KHz ultrasonic pulses and receives
 * echo from an object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

#if defined(SONAR)
STATIC_UNIT_TESTED volatile int32_t measurement = -1;
static volatile uint32_t lastMeasurementReceivedAt;
static uint32_t lastMeasurementStartedAt;
static sonarHardware_t const *sonarHardware;

#if !defined(UNIT_TEST)
static void ECHO_EXTI_IRQHandler(void)
{
    static uint32_t timing_start;
    uint32_t timing_stop;

    if (digitalIn(sonarHardware->echo_gpio, sonarHardware->echo_pin) != 0) {
        timing_start = micros();
    } else {
        timing_stop = micros();
        if (timing_stop > timing_start) {
            measurement = timing_stop - timing_start;
            lastMeasurementReceivedAt = millis();
        }
    }

    EXTI_ClearITPendingBit(sonarHardware->exti_line);
}

#if defined(CC3D)
void EXTI0_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif

#if defined(CONNEXFC)
void EXTI2_TS_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif

#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R) || defined(OLIMEXINO) || defined(SPRACINGF3)
void EXTI1_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif

#if defined(NAZE) || defined(EUSTM32F103RC) || defined(PORT103R)
void EXTI9_5_IRQHandler(void)
{
    ECHO_EXTI_IRQHandler();
}
#endif
#endif

void hcsr04_init(const sonarHardware_t *initialSonarHardware, sonarRange_t *sonarRange)
{
    sonarHardware = initialSonarHardware;
    sonarRange->maxRangeCm = HCSR04_MAX_RANGE_CM;
    sonarRange->detectionConeDeciDegrees = HCSR04_DETECTION_CONE_DECIDEGREES;
    sonarRange->detectionConeExtendedDeciDegrees = HCSR04_DETECTION_CONE_EXTENDED_DECIDEGREES;

#if !defined(UNIT_TEST)
    gpio_config_t gpio;
    EXTI_InitTypeDef EXTIInit;

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

    // trigger pin
    gpio.pin = sonarHardware->trigger_pin;
    gpio.mode = Mode_Out_PP;
    gpio.speed = Speed_2MHz;
    gpioInit(sonarHardware->trigger_gpio, &gpio);

    // echo pin
    gpio.pin = sonarHardware->echo_pin;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(sonarHardware->echo_gpio, &gpio);

#ifdef STM32F10X
    // setup external interrupt on echo pin
    if (sonarHardware->echo_gpio == GPIOA) {
        gpioExtiLineConfig(GPIO_PortSourceGPIOA, sonarHardware->exti_pin_source);
    }
    else if (sonarHardware->echo_gpio == GPIOB) {
        gpioExtiLineConfig(GPIO_PortSourceGPIOB, sonarHardware->exti_pin_source);
    }
#endif

#ifdef STM32F303xC
    if (sonarHardware->echo_gpio == GPIOA) {
        gpioExtiLineConfig(EXTI_PortSourceGPIOA, sonarHardware->exti_pin_source);
    }
    else if (sonarHardware->echo_gpio == GPIOB) {
        gpioExtiLineConfig(EXTI_PortSourceGPIOB, sonarHardware->exti_pin_source);
    }
#endif

    EXTI_ClearITPendingBit(sonarHardware->exti_line);

    EXTIInit.EXTI_Line = sonarHardware->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = sonarHardware->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_SONAR_ECHO);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    lastMeasurementStartedAt = millis() - 60; // force 1st measurement in hcsr04_get_distance()
#else
    lastMeasurementStartedAt = 0; // to avoid "unused" compiler warning
#endif
}

// measurement reading is done asynchronously, using interrupt
void hcsr04_start_reading(void)
{
#if !defined(UNIT_TEST)
    uint32_t now = millis();

    if (now < (lastMeasurementStartedAt + 100)) {
        // the repeat interval of trig signal should be greater than 100ms
        // to avoid interference between connective measurements.
        return;
    }

    lastMeasurementStartedAt = now;

#ifdef SONAR_TRIG_INVERTED
    digitalLo(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
    delayMicroseconds(11);
    digitalHi(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
#else
    digitalHi(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
    delayMicroseconds(11);
    digitalLo(sonarHardware->trigger_gpio, sonarHardware->trigger_pin);
#endif
#endif
}

/**
 * Get the distance that was measured by the last pulse, in centimeters.
 */
int32_t hcsr04_get_distance(void)
{
    uint32_t now = millis();

    if ((lastMeasurementReceivedAt > lastMeasurementStartedAt) || (now < (lastMeasurementReceivedAt + 210))) {
        /* If we received a valid pulse or might receive it soon - return latest measurement
         * Sonar is assumed to be samples at least at 5Hz rate
         *
         * The speed of sound is 340 m/s or approx. 29 microseconds per centimeter.
         * The ping travels out and back, so to find the distance of the
         * object we take half of the distance traveled.
         *
         * 340 m/s = 0.034 cm/microsecond = 29.41176471 *2 = 58.82352941 rounded to 59 
         */
         
        int32_t distance = measurement / 59;

        return distance;
    }
    else {
        return -2;
    }
}
#endif
