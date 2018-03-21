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

#include "platform.h"

#include "build_config.h"

#include "gpio.h"

#include "sound_beeper.h"

void initBeeperHardware(beeperConfig_t *config)
{
#ifndef BEEPER
    UNUSED(config);
#else
  
#if !defined(BEEP_PWM)
    gpio_config_t gpioConfig = {
        config->gpioPin,
        config->gpioMode,
        Speed_2MHz
    };

    RCC_AHBPeriphClockCmd(config->gpioPeripheral, ENABLE);

    gpioInit(config->gpioPort, &gpioConfig);
#else
    UNUSED(config);

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(BEEP_PERIPHERAL, ENABLE);

    GPIO_PinAFConfig(BEEP_GPIO, BEEP_PIN_SOURCE,  BEEP_GPIO_AF);

    /* Configuration alternate function push-pull */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = BEEP_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BEEP_GPIO, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(BEEP_TIMER_APB2_PERIPHERAL, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 370;   // 2.7kHz
    TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(BEEP_TIMER, &TIM_TimeBaseStructure);
    TIM_Cmd(BEEP_TIMER, ENABLE);

    /* PWM1 Mode configuration */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 185;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(BEEP_TIMER, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(BEEP_TIMER, TIM_OCPreload_Enable);

    TIM_CtrlPWMOutputs(BEEP_TIMER, DISABLE);
#endif
#endif
}
