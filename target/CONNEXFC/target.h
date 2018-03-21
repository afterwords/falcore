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

#define TARGET_BOARD_IDENTIFIER "CNNX" // CoNNeX

#define BOARD_HAS_VOLTAGE_DIVIDER

#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_2   // Green - PC2
#define LED0_PERIPHERAL RCC_AHBPeriph_GPIOC

#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_11  // Red - PB11
#define LED1_PERIPHERAL RCC_AHBPeriph_GPIOB

#define BEEP_PWM
#define BEEP_GPIO                   GPIOB
#define BEEP_PIN                    Pin_4
#define BEEP_PIN_SOURCE             GPIO_PinSource4
#define BEEP_GPIO_AF                GPIO_AF_1
#define BEEP_PERIPHERAL             RCC_AHBPeriph_GPIOB
#define BEEP_TIMER                  TIM16
#define BEEP_TIMER_APB2_PERIPHERAL  RCC_APB2Periph_TIM16

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2

#define SPI1_GPIO                   GPIOA
#define SPI1_GPIO_PERIPHERAL        RCC_AHBPeriph_GPIOA
#define SPI1_SCK_PIN                GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE         GPIO_PinSource5
#define SPI1_MISO_PIN               GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE        GPIO_PinSource6
#define SPI1_MOSI_GPIO              GPIOB
#define SPI1_MOSI_GPIO_PERIPHERAL   RCC_AHBPeriph_GPIOB
#define SPI1_MOSI_PIN               GPIO_Pin_5
#define SPI1_MOSI_PIN_SOURCE        GPIO_PinSource5

#define SPI2_GPIO               GPIOB
#define SPI2_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOB
#define SPI2_SCK_PIN            Pin_13
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN           Pin_14
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN           Pin_15
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15

#define USABLE_TIMER_CHANNEL_COUNT 9

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOC
#define MPU6500_CS_GPIO                  GPIOC
#define MPU6500_CS_PIN                   GPIO_Pin_0
#define MPU6500_SPI_INSTANCE             SPI1

#define M25P16_CS_GPIO_CLK_PERIPHERAL   RCC_AHBPeriph_GPIOC
#define M25P16_CS_GPIO                  GPIOC
#define M25P16_CS_PIN                   GPIO_Pin_1
#define M25P16_SPI_INSTANCE             SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define EXTI_CALLBACK_HANDLER_COUNT 1   // MPU data ready, MAG data ready
#define USE_MPU_DATA_READY_SIGNAL

#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN CW0_DEG

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN CW0_DEG

#define BARO
#define USE_BARO_MS5607
#define MS56XX_BUS          I2C_DEVICE_INT

#define MAG
#define USE_MAG_HMC5883
#define HMC5883_BUS         I2C_DEVICE_INT
#define MAG_HMC5883_ALIGN   CW0_DEG

#define USE_MAG_MAG3110
#define MAG3110_BUS         I2C_DEVICE_INT
#define MAG_MAG3110_ALIGN   CW0_DEG

#define LED0
#define LED1
#define BEEPER

#define USE_VCP
#define USE_USART1 // Conn 1 - TX (PB6) RX PB7 (AF7)
#define USE_USART2 // Input - RX (PA3)
#define USE_USART4
#define USE_USART5
#define SERIAL_PORT_COUNT 5

#define UART1_TX_PIN        GPIO_Pin_4 // PC4
#define UART1_RX_PIN        GPIO_Pin_5 // PC5
#define UART1_GPIO          GPIOC
#define UART1_GPIO_AF       GPIO_AF_7
#define UART1_TX_PINSOURCE  GPIO_PinSource4
#define UART1_RX_PINSOURCE  GPIO_PinSource5

#define UART2_TX_PIN        GPIO_Pin_2 // PA2 - Clashes with PWM6 input.
#define UART2_RX_PIN        GPIO_Pin_3 // PA3
#define UART2_GPIO          GPIOA
#define UART2_GPIO_AF       GPIO_AF_7
#define UART2_TX_PINSOURCE  GPIO_PinSource2
#define UART2_RX_PINSOURCE  GPIO_PinSource3

#define USE_I2C
#define I2C_DEVICE_INT (I2CDEV_1) // PB6/SCL, PB7/SDA
#define I2C_DEVICE_EXT (I2CDEV_2)

#define USE_ADC

#define ADC_INSTANCE                ADC1
#define ADC_AHB_PERIPHERAL          RCC_AHBPeriph_DMA1
#define ADC_DMA_CHANNEL             DMA1_Channel1

#define VBAT_ADC_GPIO               GPIOF
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_4
#define VBAT_ADC_CHANNEL            ADC_Channel_5

#define RSSI_ADC_GPIO               GPIOA
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_1
#define RSSI_ADC_CHANNEL            ADC_Channel_2

#define SONAR

//#define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT

#define RND 0
#define RTF 1
#define ARF 2

#define BLACKBOX
#define USE_CLI

#if BUILDTYPE == ARF
    #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#elif BUILDTYPE == RTF
    #define ENABLE_BLACKBOX_LOGGING_ON_SPIFLASH_BY_DEFAULT
#else 
    #define INCLUDE_FALCORE_CLI_PARAMS
    #define GPS
#endif

#define SERIAL_RX
#define TELEMETRY
#define USE_SERVOS

#define LED_STRIP
#define LED_STRIP_TIMER TIM1

#define USE_LED_STRIP_ON_DMA1_CHANNEL2
#define WS2811_GPIO                     GPIOA
#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
#define WS2811_GPIO_AF                  GPIO_AF_6
#define WS2811_PIN                      GPIO_Pin_8
#define WS2811_PIN_SOURCE               GPIO_PinSource8
#define WS2811_TIMER                    TIM1
#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM1
#define WS2811_DMA_CHANNEL              DMA1_Channel2
#define WS2811_IRQ                      DMA1_Channel2_IRQn

// USART2, PA3
#define SPEKTRUM_BIND
#define BIND_PORT GPIOA
#define BIND_PIN Pin_3

#define USE_SERIAL_4WAY_BLHELI_INTERFACE

// MAVlink options
//#define USE_PROSIGHT_IGNITION
//#define MAVLINK_DEBUG_MESSAGE

