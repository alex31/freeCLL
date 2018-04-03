/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#pragma once


/*
 * Setup for STMicroelectronics STM32 Nucleo32-L432KC board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_NUCLEO32_L432KC
#define BOARD_NAME                  "STMicroelectronics STM32 Nucleo32-L432KC"

/*
 * Board oscillators-related settings.
 * NOTE: HSE not fitted.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (2U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                0U
#endif

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32L432xx

/*
 * IO pins assignments.
 */
/*
 * IO pins assignments.
 */
#define	PA00_RCC_CK_IN                 0U
#define	PA01                           1U
#define	PA02_SHELL_TX                  2U
#define	PA03_CASTEL_ICU2               3U
#define	PA04_TEST_PULSE                4U
#define	PA05_CASTEL_ICU1               5U
#define	PA06                           6U
#define	PA07                           7U
#define	PA08_CASTEL_PWM1               8U
#define	PA09                           9U
#define	PA10                           10U
#define	PA11_CASTEL_PWM2               11U
#define	PA12                           12U
#define	PA13_SWDIO                     13U
#define	PA14_SWCLK                     14U
#define	PA15_SHELL_RX                  15U

#define	PB00_INPUT_BUT                 0U
#define	PB01                           1U
#define	PB02                           2U
#define	PB03_LED_GREEN                 3U
#define	PB04                           4U
#define	PB05                           5U
#define	PB06_CASTEL_TX                 6U
#define	PB07_CASTEL_RX                 7U
#define	PB08                           8U
#define	PB09                           9U
#define	PB10                           10U
#define	PB11                           11U
#define	PB12                           12U
#define	PB13                           13U
#define	PB14                           14U
#define	PB15                           15U

#define	PC00                           0U
#define	PC01                           1U
#define	PC02                           2U
#define	PC03                           3U
#define	PC04                           4U
#define	PC05                           5U
#define	PC06                           6U
#define	PC07                           7U
#define	PC08                           8U
#define	PC09                           9U
#define	PC10                           10U
#define	PC11                           11U
#define	PC12                           12U
#define	PC13                           13U
#define	PC14_RCC_OSC32_IN              14U
#define	PC15_RCC_OSC32_OUT             15U

#define	PD00                           0U
#define	PD01                           1U
#define	PD02                           2U
#define	PD03                           3U
#define	PD04                           4U
#define	PD05                           5U
#define	PD06                           6U
#define	PD07                           7U
#define	PD08                           8U
#define	PD09                           9U
#define	PD10                           10U
#define	PD11                           11U
#define	PD12                           12U
#define	PD13                           13U
#define	PD14                           14U
#define	PD15                           15U

#define	PH00                           0U
#define	PH01                           1U
#define	PH02                           2U
#define	PH03                           3U
#define	PH04                           4U
#define	PH05                           5U
#define	PH06                           6U
#define	PH07                           7U
#define	PH08                           8U
#define	PH09                           9U
#define	PH10                           10U
#define	PH11                           11U
#define	PH12                           12U
#define	PH13                           13U
#define	PH14                           14U
#define	PH15                           15U

#define	PE00                           0U
#define	PE01                           1U
#define	PE02                           2U
#define	PE03                           3U
#define	PE04                           4U
#define	PE05                           5U
#define	PE06                           6U
#define	PE07                           7U
#define	PE08                           8U
#define	PE09                           9U
#define	PE10                           10U
#define	PE11                           11U
#define	PE12                           12U
#define	PE13                           13U
#define	PE14                           14U
#define	PE15                           15U

/*
 * IO lines assignments.
 */
#define	LINE_RCC_CK_IN                 PAL_LINE(GPIOA, 0U)
#define	LINE_SHELL_TX                  PAL_LINE(GPIOA, 2U)
#define	LINE_CASTEL_ICU2               PAL_LINE(GPIOA, 3U)
#define	LINE_TEST_PULSE                PAL_LINE(GPIOA, 4U)
#define	LINE_CASTEL_ICU1               PAL_LINE(GPIOA, 5U)
#define	LINE_CASTEL_PWM1               PAL_LINE(GPIOA, 8U)
#define	LINE_CASTEL_PWM2               PAL_LINE(GPIOA, 11U)
#define	LINE_SWDIO                     PAL_LINE(GPIOA, 13U)
#define	LINE_SWCLK                     PAL_LINE(GPIOA, 14U)
#define	LINE_SHELL_RX                  PAL_LINE(GPIOA, 15U)

#define	LINE_INPUT_BUT                 PAL_LINE(GPIOB, 0U)
#define	LINE_LED_GREEN                 PAL_LINE(GPIOB, 3U)
#define	LINE_CASTEL_TX                 PAL_LINE(GPIOB, 6U)
#define	LINE_CASTEL_RX                 PAL_LINE(GPIOB, 7U)

#define	LINE_RCC_OSC32_IN              PAL_LINE(GPIOC, 14U)
#define	LINE_RCC_OSC32_OUT             PAL_LINE(GPIOC, 15U)


/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LEVEL_LOW(n)        (0U << (n))
#define PIN_ODR_LEVEL_HIGH(n)       (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_SPEED_VERYLOW(n) (0U << ((n) * 2U))
#define PIN_OSPEED_SPEED_LOW(n)     (1U << ((n) * 2U))
#define PIN_OSPEED_SPEED_MEDIUM(n)  (2U << ((n) * 2U))
#define PIN_OSPEED_SPEED_HIGH(n)    (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

#define PIN_ASCR_DISABLED(n)        (0U << (n))
#define PIN_ASCR_ENABLED(n)         (1U << (n))
#define PIN_LOCKR_DISABLED(n)       (0U << (n))
#define PIN_LOCKR_ENABLED(n)        (1U << (n))

#define VAL_GPIOA_MODER                 (PIN_MODE_ALTERNATE(PA00_RCC_CK_IN) | \
					 PIN_MODE_ANALOG(PA01) | \
					 PIN_MODE_ALTERNATE(PA02_SHELL_TX) | \
					 PIN_MODE_ALTERNATE(PA03_CASTEL_ICU2) | \
					 PIN_MODE_OUTPUT(PA04_TEST_PULSE) | \
					 PIN_MODE_ALTERNATE(PA05_CASTEL_ICU1) | \
					 PIN_MODE_ANALOG(PA06) | \
					 PIN_MODE_ANALOG(PA07) | \
					 PIN_MODE_ALTERNATE(PA08_CASTEL_PWM1) | \
					 PIN_MODE_ANALOG(PA09) | \
					 PIN_MODE_ANALOG(PA10) | \
					 PIN_MODE_ALTERNATE(PA11_CASTEL_PWM2) | \
					 PIN_MODE_ANALOG(PA12) | \
					 PIN_MODE_ALTERNATE(PA13_SWDIO) | \
					 PIN_MODE_ALTERNATE(PA14_SWCLK) | \
					 PIN_MODE_ALTERNATE(PA15_SHELL_RX))

#define VAL_GPIOA_OTYPER                (PIN_OTYPE_PUSHPULL(PA00_RCC_CK_IN) | \
					 PIN_OTYPE_PUSHPULL(PA01) | \
					 PIN_OTYPE_PUSHPULL(PA02_SHELL_TX) | \
					 PIN_OTYPE_PUSHPULL(PA03_CASTEL_ICU2) | \
					 PIN_OTYPE_OPENDRAIN(PA04_TEST_PULSE) | \
					 PIN_OTYPE_PUSHPULL(PA05_CASTEL_ICU1) | \
					 PIN_OTYPE_PUSHPULL(PA06) | \
					 PIN_OTYPE_PUSHPULL(PA07) | \
					 PIN_OTYPE_PUSHPULL(PA08_CASTEL_PWM1) | \
					 PIN_OTYPE_PUSHPULL(PA09) | \
					 PIN_OTYPE_PUSHPULL(PA10) | \
					 PIN_OTYPE_PUSHPULL(PA11_CASTEL_PWM2) | \
					 PIN_OTYPE_PUSHPULL(PA12) | \
					 PIN_OTYPE_PUSHPULL(PA13_SWDIO) | \
					 PIN_OTYPE_PUSHPULL(PA14_SWCLK) | \
					 PIN_OTYPE_PUSHPULL(PA15_SHELL_RX))

#define VAL_GPIOA_OSPEEDR               (PIN_OSPEED_SPEED_HIGH(PA00_RCC_CK_IN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA01) | \
					 PIN_OSPEED_SPEED_HIGH(PA02_SHELL_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PA03_CASTEL_ICU2) | \
					 PIN_OSPEED_SPEED_HIGH(PA04_TEST_PULSE) | \
					 PIN_OSPEED_SPEED_HIGH(PA05_CASTEL_ICU1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA07) | \
					 PIN_OSPEED_SPEED_HIGH(PA08_CASTEL_PWM1) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA10) | \
					 PIN_OSPEED_SPEED_HIGH(PA11_CASTEL_PWM2) | \
					 PIN_OSPEED_SPEED_VERYLOW(PA12) | \
					 PIN_OSPEED_SPEED_HIGH(PA13_SWDIO) | \
					 PIN_OSPEED_SPEED_HIGH(PA14_SWCLK) | \
					 PIN_OSPEED_SPEED_HIGH(PA15_SHELL_RX))

#define VAL_GPIOA_PUPDR                 (PIN_PUPDR_FLOATING(PA00_RCC_CK_IN) | \
					 PIN_PUPDR_FLOATING(PA01) | \
					 PIN_PUPDR_FLOATING(PA02_SHELL_TX) | \
					 PIN_PUPDR_FLOATING(PA03_CASTEL_ICU2) | \
					 PIN_PUPDR_PULLUP(PA04_TEST_PULSE) | \
					 PIN_PUPDR_FLOATING(PA05_CASTEL_ICU1) | \
					 PIN_PUPDR_FLOATING(PA06) | \
					 PIN_PUPDR_FLOATING(PA07) | \
					 PIN_PUPDR_FLOATING(PA08_CASTEL_PWM1) | \
					 PIN_PUPDR_FLOATING(PA09) | \
					 PIN_PUPDR_FLOATING(PA10) | \
					 PIN_PUPDR_FLOATING(PA11_CASTEL_PWM2) | \
					 PIN_PUPDR_FLOATING(PA12) | \
					 PIN_PUPDR_PULLUP(PA13_SWDIO) | \
					 PIN_PUPDR_PULLUP(PA14_SWCLK) | \
					 PIN_PUPDR_FLOATING(PA15_SHELL_RX))

#define VAL_GPIOA_ODR                   (PIN_ODR_LEVEL_HIGH(PA00_RCC_CK_IN) | \
					 PIN_ODR_LEVEL_HIGH(PA01) | \
					 PIN_ODR_LEVEL_HIGH(PA02_SHELL_TX) | \
					 PIN_ODR_LEVEL_HIGH(PA03_CASTEL_ICU2) | \
					 PIN_ODR_LEVEL_HIGH(PA04_TEST_PULSE) | \
					 PIN_ODR_LEVEL_HIGH(PA05_CASTEL_ICU1) | \
					 PIN_ODR_LEVEL_HIGH(PA06) | \
					 PIN_ODR_LEVEL_HIGH(PA07) | \
					 PIN_ODR_LEVEL_LOW(PA08_CASTEL_PWM1) | \
					 PIN_ODR_LEVEL_HIGH(PA09) | \
					 PIN_ODR_LEVEL_HIGH(PA10) | \
					 PIN_ODR_LEVEL_LOW(PA11_CASTEL_PWM2) | \
					 PIN_ODR_LEVEL_HIGH(PA12) | \
					 PIN_ODR_LEVEL_HIGH(PA13_SWDIO) | \
					 PIN_ODR_LEVEL_HIGH(PA14_SWCLK) | \
					 PIN_ODR_LEVEL_HIGH(PA15_SHELL_RX))

#define VAL_GPIOA_AFRL			(PIN_AFIO_AF(PA00_RCC_CK_IN, 0) | \
					 PIN_AFIO_AF(PA01, 0) | \
					 PIN_AFIO_AF(PA02_SHELL_TX, 7) | \
					 PIN_AFIO_AF(PA03_CASTEL_ICU2, 14) | \
					 PIN_AFIO_AF(PA04_TEST_PULSE, 0) | \
					 PIN_AFIO_AF(PA05_CASTEL_ICU1, 1) | \
					 PIN_AFIO_AF(PA06, 0) | \
					 PIN_AFIO_AF(PA07, 0))

#define VAL_GPIOA_AFRH			(PIN_AFIO_AF(PA08_CASTEL_PWM1, 1) | \
					 PIN_AFIO_AF(PA09, 0) | \
					 PIN_AFIO_AF(PA10, 0) | \
					 PIN_AFIO_AF(PA11_CASTEL_PWM2, 1) | \
					 PIN_AFIO_AF(PA12, 0) | \
					 PIN_AFIO_AF(PA13_SWDIO, 0) | \
					 PIN_AFIO_AF(PA14_SWCLK, 0) | \
					 PIN_AFIO_AF(PA15_SHELL_RX, 3))

#define VAL_GPIOA_ASCR                  (PIN_ASCR_DISABLED(PA00_RCC_CK_IN) | \
					 PIN_ASCR_DISABLED(PA01) | \
					 PIN_ASCR_DISABLED(PA02_SHELL_TX) | \
					 PIN_ASCR_DISABLED(PA03_CASTEL_ICU2) | \
					 PIN_ASCR_DISABLED(PA04_TEST_PULSE) | \
					 PIN_ASCR_DISABLED(PA05_CASTEL_ICU1) | \
					 PIN_ASCR_DISABLED(PA06) | \
					 PIN_ASCR_DISABLED(PA07) | \
					 PIN_ASCR_DISABLED(PA08_CASTEL_PWM1) | \
					 PIN_ASCR_DISABLED(PA09) | \
					 PIN_ASCR_DISABLED(PA10) | \
					 PIN_ASCR_DISABLED(PA11_CASTEL_PWM2) | \
					 PIN_ASCR_DISABLED(PA12) | \
					 PIN_ASCR_DISABLED(PA13_SWDIO) | \
					 PIN_ASCR_DISABLED(PA14_SWCLK) | \
					 PIN_ASCR_DISABLED(PA15_SHELL_RX))

#define VAL_GPIOA_LOCKR                 (PIN_LOCKR_DISABLED(PA00_RCC_CK_IN) | \
					 PIN_LOCKR_DISABLED(PA01) | \
					 PIN_LOCKR_DISABLED(PA02_SHELL_TX) | \
					 PIN_LOCKR_DISABLED(PA03_CASTEL_ICU2) | \
					 PIN_LOCKR_DISABLED(PA04_TEST_PULSE) | \
					 PIN_LOCKR_DISABLED(PA05_CASTEL_ICU1) | \
					 PIN_LOCKR_DISABLED(PA06) | \
					 PIN_LOCKR_DISABLED(PA07) | \
					 PIN_LOCKR_DISABLED(PA08_CASTEL_PWM1) | \
					 PIN_LOCKR_DISABLED(PA09) | \
					 PIN_LOCKR_DISABLED(PA10) | \
					 PIN_LOCKR_DISABLED(PA11_CASTEL_PWM2) | \
					 PIN_LOCKR_DISABLED(PA12) | \
					 PIN_LOCKR_DISABLED(PA13_SWDIO) | \
					 PIN_LOCKR_DISABLED(PA14_SWCLK) | \
					 PIN_LOCKR_DISABLED(PA15_SHELL_RX))

#define VAL_GPIOB_MODER                 (PIN_MODE_INPUT(PB00_INPUT_BUT) | \
					 PIN_MODE_ANALOG(PB01) | \
					 PIN_MODE_ANALOG(PB02) | \
					 PIN_MODE_OUTPUT(PB03_LED_GREEN) | \
					 PIN_MODE_ANALOG(PB04) | \
					 PIN_MODE_ANALOG(PB05) | \
					 PIN_MODE_ALTERNATE(PB06_CASTEL_TX) | \
					 PIN_MODE_ALTERNATE(PB07_CASTEL_RX) | \
					 PIN_MODE_ANALOG(PB08) | \
					 PIN_MODE_ANALOG(PB09) | \
					 PIN_MODE_ANALOG(PB10) | \
					 PIN_MODE_ANALOG(PB11) | \
					 PIN_MODE_ANALOG(PB12) | \
					 PIN_MODE_ANALOG(PB13) | \
					 PIN_MODE_ANALOG(PB14) | \
					 PIN_MODE_ANALOG(PB15))

#define VAL_GPIOB_OTYPER                (PIN_OTYPE_OPENDRAIN(PB00_INPUT_BUT) | \
					 PIN_OTYPE_PUSHPULL(PB01) | \
					 PIN_OTYPE_PUSHPULL(PB02) | \
					 PIN_OTYPE_PUSHPULL(PB03_LED_GREEN) | \
					 PIN_OTYPE_PUSHPULL(PB04) | \
					 PIN_OTYPE_PUSHPULL(PB05) | \
					 PIN_OTYPE_PUSHPULL(PB06_CASTEL_TX) | \
					 PIN_OTYPE_PUSHPULL(PB07_CASTEL_RX) | \
					 PIN_OTYPE_PUSHPULL(PB08) | \
					 PIN_OTYPE_PUSHPULL(PB09) | \
					 PIN_OTYPE_PUSHPULL(PB10) | \
					 PIN_OTYPE_PUSHPULL(PB11) | \
					 PIN_OTYPE_PUSHPULL(PB12) | \
					 PIN_OTYPE_PUSHPULL(PB13) | \
					 PIN_OTYPE_PUSHPULL(PB14) | \
					 PIN_OTYPE_PUSHPULL(PB15))

#define VAL_GPIOB_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PB00_INPUT_BUT) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB03_LED_GREEN) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB05) | \
					 PIN_OSPEED_SPEED_HIGH(PB06_CASTEL_TX) | \
					 PIN_OSPEED_SPEED_HIGH(PB07_CASTEL_RX) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PB15))

#define VAL_GPIOB_PUPDR                 (PIN_PUPDR_PULLUP(PB00_INPUT_BUT) | \
					 PIN_PUPDR_FLOATING(PB01) | \
					 PIN_PUPDR_FLOATING(PB02) | \
					 PIN_PUPDR_FLOATING(PB03_LED_GREEN) | \
					 PIN_PUPDR_FLOATING(PB04) | \
					 PIN_PUPDR_FLOATING(PB05) | \
					 PIN_PUPDR_FLOATING(PB06_CASTEL_TX) | \
					 PIN_PUPDR_FLOATING(PB07_CASTEL_RX) | \
					 PIN_PUPDR_FLOATING(PB08) | \
					 PIN_PUPDR_FLOATING(PB09) | \
					 PIN_PUPDR_FLOATING(PB10) | \
					 PIN_PUPDR_FLOATING(PB11) | \
					 PIN_PUPDR_FLOATING(PB12) | \
					 PIN_PUPDR_FLOATING(PB13) | \
					 PIN_PUPDR_FLOATING(PB14) | \
					 PIN_PUPDR_FLOATING(PB15))

#define VAL_GPIOB_ODR                   (PIN_ODR_LEVEL_LOW(PB00_INPUT_BUT) | \
					 PIN_ODR_LEVEL_HIGH(PB01) | \
					 PIN_ODR_LEVEL_HIGH(PB02) | \
					 PIN_ODR_LEVEL_LOW(PB03_LED_GREEN) | \
					 PIN_ODR_LEVEL_HIGH(PB04) | \
					 PIN_ODR_LEVEL_HIGH(PB05) | \
					 PIN_ODR_LEVEL_HIGH(PB06_CASTEL_TX) | \
					 PIN_ODR_LEVEL_HIGH(PB07_CASTEL_RX) | \
					 PIN_ODR_LEVEL_HIGH(PB08) | \
					 PIN_ODR_LEVEL_HIGH(PB09) | \
					 PIN_ODR_LEVEL_HIGH(PB10) | \
					 PIN_ODR_LEVEL_HIGH(PB11) | \
					 PIN_ODR_LEVEL_HIGH(PB12) | \
					 PIN_ODR_LEVEL_HIGH(PB13) | \
					 PIN_ODR_LEVEL_HIGH(PB14) | \
					 PIN_ODR_LEVEL_HIGH(PB15))

#define VAL_GPIOB_AFRL			(PIN_AFIO_AF(PB00_INPUT_BUT, 0) | \
					 PIN_AFIO_AF(PB01, 0) | \
					 PIN_AFIO_AF(PB02, 0) | \
					 PIN_AFIO_AF(PB03_LED_GREEN, 0) | \
					 PIN_AFIO_AF(PB04, 0) | \
					 PIN_AFIO_AF(PB05, 0) | \
					 PIN_AFIO_AF(PB06_CASTEL_TX, 7) | \
					 PIN_AFIO_AF(PB07_CASTEL_RX, 7))

#define VAL_GPIOB_AFRH			(PIN_AFIO_AF(PB08, 0) | \
					 PIN_AFIO_AF(PB09, 0) | \
					 PIN_AFIO_AF(PB10, 0) | \
					 PIN_AFIO_AF(PB11, 0) | \
					 PIN_AFIO_AF(PB12, 0) | \
					 PIN_AFIO_AF(PB13, 0) | \
					 PIN_AFIO_AF(PB14, 0) | \
					 PIN_AFIO_AF(PB15, 0))

#define VAL_GPIOB_ASCR                  (PIN_ASCR_DISABLED(PB00_INPUT_BUT) | \
					 PIN_ASCR_DISABLED(PB01) | \
					 PIN_ASCR_DISABLED(PB02) | \
					 PIN_ASCR_DISABLED(PB03_LED_GREEN) | \
					 PIN_ASCR_DISABLED(PB04) | \
					 PIN_ASCR_DISABLED(PB05) | \
					 PIN_ASCR_DISABLED(PB06_CASTEL_TX) | \
					 PIN_ASCR_DISABLED(PB07_CASTEL_RX) | \
					 PIN_ASCR_DISABLED(PB08) | \
					 PIN_ASCR_DISABLED(PB09) | \
					 PIN_ASCR_DISABLED(PB10) | \
					 PIN_ASCR_DISABLED(PB11) | \
					 PIN_ASCR_DISABLED(PB12) | \
					 PIN_ASCR_DISABLED(PB13) | \
					 PIN_ASCR_DISABLED(PB14) | \
					 PIN_ASCR_DISABLED(PB15))

#define VAL_GPIOB_LOCKR                 (PIN_LOCKR_DISABLED(PB00_INPUT_BUT) | \
					 PIN_LOCKR_DISABLED(PB01) | \
					 PIN_LOCKR_DISABLED(PB02) | \
					 PIN_LOCKR_DISABLED(PB03_LED_GREEN) | \
					 PIN_LOCKR_DISABLED(PB04) | \
					 PIN_LOCKR_DISABLED(PB05) | \
					 PIN_LOCKR_DISABLED(PB06_CASTEL_TX) | \
					 PIN_LOCKR_DISABLED(PB07_CASTEL_RX) | \
					 PIN_LOCKR_DISABLED(PB08) | \
					 PIN_LOCKR_DISABLED(PB09) | \
					 PIN_LOCKR_DISABLED(PB10) | \
					 PIN_LOCKR_DISABLED(PB11) | \
					 PIN_LOCKR_DISABLED(PB12) | \
					 PIN_LOCKR_DISABLED(PB13) | \
					 PIN_LOCKR_DISABLED(PB14) | \
					 PIN_LOCKR_DISABLED(PB15))

#define VAL_GPIOC_MODER                 (PIN_MODE_ANALOG(PC00) | \
					 PIN_MODE_ANALOG(PC01) | \
					 PIN_MODE_ANALOG(PC02) | \
					 PIN_MODE_ANALOG(PC03) | \
					 PIN_MODE_ANALOG(PC04) | \
					 PIN_MODE_ANALOG(PC05) | \
					 PIN_MODE_ANALOG(PC06) | \
					 PIN_MODE_ANALOG(PC07) | \
					 PIN_MODE_ANALOG(PC08) | \
					 PIN_MODE_ANALOG(PC09) | \
					 PIN_MODE_ANALOG(PC10) | \
					 PIN_MODE_ANALOG(PC11) | \
					 PIN_MODE_ANALOG(PC12) | \
					 PIN_MODE_ANALOG(PC13) | \
					 PIN_MODE_ALTERNATE(PC14_RCC_OSC32_IN) | \
					 PIN_MODE_ALTERNATE(PC15_RCC_OSC32_OUT))

#define VAL_GPIOC_OTYPER                (PIN_OTYPE_PUSHPULL(PC00) | \
					 PIN_OTYPE_PUSHPULL(PC01) | \
					 PIN_OTYPE_PUSHPULL(PC02) | \
					 PIN_OTYPE_PUSHPULL(PC03) | \
					 PIN_OTYPE_PUSHPULL(PC04) | \
					 PIN_OTYPE_PUSHPULL(PC05) | \
					 PIN_OTYPE_PUSHPULL(PC06) | \
					 PIN_OTYPE_PUSHPULL(PC07) | \
					 PIN_OTYPE_PUSHPULL(PC08) | \
					 PIN_OTYPE_PUSHPULL(PC09) | \
					 PIN_OTYPE_PUSHPULL(PC10) | \
					 PIN_OTYPE_PUSHPULL(PC11) | \
					 PIN_OTYPE_PUSHPULL(PC12) | \
					 PIN_OTYPE_PUSHPULL(PC13) | \
					 PIN_OTYPE_PUSHPULL(PC14_RCC_OSC32_IN) | \
					 PIN_OTYPE_PUSHPULL(PC15_RCC_OSC32_OUT))

#define VAL_GPIOC_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PC00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PC13) | \
					 PIN_OSPEED_SPEED_HIGH(PC14_RCC_OSC32_IN) | \
					 PIN_OSPEED_SPEED_HIGH(PC15_RCC_OSC32_OUT))

#define VAL_GPIOC_PUPDR                 (PIN_PUPDR_FLOATING(PC00) | \
					 PIN_PUPDR_FLOATING(PC01) | \
					 PIN_PUPDR_FLOATING(PC02) | \
					 PIN_PUPDR_FLOATING(PC03) | \
					 PIN_PUPDR_FLOATING(PC04) | \
					 PIN_PUPDR_FLOATING(PC05) | \
					 PIN_PUPDR_FLOATING(PC06) | \
					 PIN_PUPDR_FLOATING(PC07) | \
					 PIN_PUPDR_FLOATING(PC08) | \
					 PIN_PUPDR_FLOATING(PC09) | \
					 PIN_PUPDR_FLOATING(PC10) | \
					 PIN_PUPDR_FLOATING(PC11) | \
					 PIN_PUPDR_FLOATING(PC12) | \
					 PIN_PUPDR_FLOATING(PC13) | \
					 PIN_PUPDR_FLOATING(PC14_RCC_OSC32_IN) | \
					 PIN_PUPDR_FLOATING(PC15_RCC_OSC32_OUT))

#define VAL_GPIOC_ODR                   (PIN_ODR_LEVEL_HIGH(PC00) | \
					 PIN_ODR_LEVEL_HIGH(PC01) | \
					 PIN_ODR_LEVEL_HIGH(PC02) | \
					 PIN_ODR_LEVEL_HIGH(PC03) | \
					 PIN_ODR_LEVEL_HIGH(PC04) | \
					 PIN_ODR_LEVEL_HIGH(PC05) | \
					 PIN_ODR_LEVEL_HIGH(PC06) | \
					 PIN_ODR_LEVEL_HIGH(PC07) | \
					 PIN_ODR_LEVEL_HIGH(PC08) | \
					 PIN_ODR_LEVEL_HIGH(PC09) | \
					 PIN_ODR_LEVEL_HIGH(PC10) | \
					 PIN_ODR_LEVEL_HIGH(PC11) | \
					 PIN_ODR_LEVEL_HIGH(PC12) | \
					 PIN_ODR_LEVEL_HIGH(PC13) | \
					 PIN_ODR_LEVEL_HIGH(PC14_RCC_OSC32_IN) | \
					 PIN_ODR_LEVEL_HIGH(PC15_RCC_OSC32_OUT))

#define VAL_GPIOC_AFRL			(PIN_AFIO_AF(PC00, 0) | \
					 PIN_AFIO_AF(PC01, 0) | \
					 PIN_AFIO_AF(PC02, 0) | \
					 PIN_AFIO_AF(PC03, 0) | \
					 PIN_AFIO_AF(PC04, 0) | \
					 PIN_AFIO_AF(PC05, 0) | \
					 PIN_AFIO_AF(PC06, 0) | \
					 PIN_AFIO_AF(PC07, 0))

#define VAL_GPIOC_AFRH			(PIN_AFIO_AF(PC08, 0) | \
					 PIN_AFIO_AF(PC09, 0) | \
					 PIN_AFIO_AF(PC10, 0) | \
					 PIN_AFIO_AF(PC11, 0) | \
					 PIN_AFIO_AF(PC12, 0) | \
					 PIN_AFIO_AF(PC13, 0) | \
					 PIN_AFIO_AF(PC14_RCC_OSC32_IN, 0) | \
					 PIN_AFIO_AF(PC15_RCC_OSC32_OUT, 0))

#define VAL_GPIOC_ASCR                  (PIN_ASCR_DISABLED(PC00) | \
					 PIN_ASCR_DISABLED(PC01) | \
					 PIN_ASCR_DISABLED(PC02) | \
					 PIN_ASCR_DISABLED(PC03) | \
					 PIN_ASCR_DISABLED(PC04) | \
					 PIN_ASCR_DISABLED(PC05) | \
					 PIN_ASCR_DISABLED(PC06) | \
					 PIN_ASCR_DISABLED(PC07) | \
					 PIN_ASCR_DISABLED(PC08) | \
					 PIN_ASCR_DISABLED(PC09) | \
					 PIN_ASCR_DISABLED(PC10) | \
					 PIN_ASCR_DISABLED(PC11) | \
					 PIN_ASCR_DISABLED(PC12) | \
					 PIN_ASCR_DISABLED(PC13) | \
					 PIN_ASCR_DISABLED(PC14_RCC_OSC32_IN) | \
					 PIN_ASCR_DISABLED(PC15_RCC_OSC32_OUT))

#define VAL_GPIOC_LOCKR                 (PIN_LOCKR_DISABLED(PC00) | \
					 PIN_LOCKR_DISABLED(PC01) | \
					 PIN_LOCKR_DISABLED(PC02) | \
					 PIN_LOCKR_DISABLED(PC03) | \
					 PIN_LOCKR_DISABLED(PC04) | \
					 PIN_LOCKR_DISABLED(PC05) | \
					 PIN_LOCKR_DISABLED(PC06) | \
					 PIN_LOCKR_DISABLED(PC07) | \
					 PIN_LOCKR_DISABLED(PC08) | \
					 PIN_LOCKR_DISABLED(PC09) | \
					 PIN_LOCKR_DISABLED(PC10) | \
					 PIN_LOCKR_DISABLED(PC11) | \
					 PIN_LOCKR_DISABLED(PC12) | \
					 PIN_LOCKR_DISABLED(PC13) | \
					 PIN_LOCKR_DISABLED(PC14_RCC_OSC32_IN) | \
					 PIN_LOCKR_DISABLED(PC15_RCC_OSC32_OUT))

#define VAL_GPIOD_MODER                 (PIN_MODE_ANALOG(PD00) | \
					 PIN_MODE_ANALOG(PD01) | \
					 PIN_MODE_ANALOG(PD02) | \
					 PIN_MODE_ANALOG(PD03) | \
					 PIN_MODE_ANALOG(PD04) | \
					 PIN_MODE_ANALOG(PD05) | \
					 PIN_MODE_ANALOG(PD06) | \
					 PIN_MODE_ANALOG(PD07) | \
					 PIN_MODE_ANALOG(PD08) | \
					 PIN_MODE_ANALOG(PD09) | \
					 PIN_MODE_ANALOG(PD10) | \
					 PIN_MODE_ANALOG(PD11) | \
					 PIN_MODE_ANALOG(PD12) | \
					 PIN_MODE_ANALOG(PD13) | \
					 PIN_MODE_ANALOG(PD14) | \
					 PIN_MODE_ANALOG(PD15))

#define VAL_GPIOD_OTYPER                (PIN_OTYPE_PUSHPULL(PD00) | \
					 PIN_OTYPE_PUSHPULL(PD01) | \
					 PIN_OTYPE_PUSHPULL(PD02) | \
					 PIN_OTYPE_PUSHPULL(PD03) | \
					 PIN_OTYPE_PUSHPULL(PD04) | \
					 PIN_OTYPE_PUSHPULL(PD05) | \
					 PIN_OTYPE_PUSHPULL(PD06) | \
					 PIN_OTYPE_PUSHPULL(PD07) | \
					 PIN_OTYPE_PUSHPULL(PD08) | \
					 PIN_OTYPE_PUSHPULL(PD09) | \
					 PIN_OTYPE_PUSHPULL(PD10) | \
					 PIN_OTYPE_PUSHPULL(PD11) | \
					 PIN_OTYPE_PUSHPULL(PD12) | \
					 PIN_OTYPE_PUSHPULL(PD13) | \
					 PIN_OTYPE_PUSHPULL(PD14) | \
					 PIN_OTYPE_PUSHPULL(PD15))

#define VAL_GPIOD_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PD00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PD15))

#define VAL_GPIOD_PUPDR                 (PIN_PUPDR_FLOATING(PD00) | \
					 PIN_PUPDR_FLOATING(PD01) | \
					 PIN_PUPDR_FLOATING(PD02) | \
					 PIN_PUPDR_FLOATING(PD03) | \
					 PIN_PUPDR_FLOATING(PD04) | \
					 PIN_PUPDR_FLOATING(PD05) | \
					 PIN_PUPDR_FLOATING(PD06) | \
					 PIN_PUPDR_FLOATING(PD07) | \
					 PIN_PUPDR_FLOATING(PD08) | \
					 PIN_PUPDR_FLOATING(PD09) | \
					 PIN_PUPDR_FLOATING(PD10) | \
					 PIN_PUPDR_FLOATING(PD11) | \
					 PIN_PUPDR_FLOATING(PD12) | \
					 PIN_PUPDR_FLOATING(PD13) | \
					 PIN_PUPDR_FLOATING(PD14) | \
					 PIN_PUPDR_FLOATING(PD15))

#define VAL_GPIOD_ODR                   (PIN_ODR_LEVEL_HIGH(PD00) | \
					 PIN_ODR_LEVEL_HIGH(PD01) | \
					 PIN_ODR_LEVEL_HIGH(PD02) | \
					 PIN_ODR_LEVEL_HIGH(PD03) | \
					 PIN_ODR_LEVEL_HIGH(PD04) | \
					 PIN_ODR_LEVEL_HIGH(PD05) | \
					 PIN_ODR_LEVEL_HIGH(PD06) | \
					 PIN_ODR_LEVEL_HIGH(PD07) | \
					 PIN_ODR_LEVEL_HIGH(PD08) | \
					 PIN_ODR_LEVEL_HIGH(PD09) | \
					 PIN_ODR_LEVEL_HIGH(PD10) | \
					 PIN_ODR_LEVEL_HIGH(PD11) | \
					 PIN_ODR_LEVEL_HIGH(PD12) | \
					 PIN_ODR_LEVEL_HIGH(PD13) | \
					 PIN_ODR_LEVEL_HIGH(PD14) | \
					 PIN_ODR_LEVEL_HIGH(PD15))

#define VAL_GPIOD_AFRL			(PIN_AFIO_AF(PD00, 0) | \
					 PIN_AFIO_AF(PD01, 0) | \
					 PIN_AFIO_AF(PD02, 0) | \
					 PIN_AFIO_AF(PD03, 0) | \
					 PIN_AFIO_AF(PD04, 0) | \
					 PIN_AFIO_AF(PD05, 0) | \
					 PIN_AFIO_AF(PD06, 0) | \
					 PIN_AFIO_AF(PD07, 0))

#define VAL_GPIOD_AFRH			(PIN_AFIO_AF(PD08, 0) | \
					 PIN_AFIO_AF(PD09, 0) | \
					 PIN_AFIO_AF(PD10, 0) | \
					 PIN_AFIO_AF(PD11, 0) | \
					 PIN_AFIO_AF(PD12, 0) | \
					 PIN_AFIO_AF(PD13, 0) | \
					 PIN_AFIO_AF(PD14, 0) | \
					 PIN_AFIO_AF(PD15, 0))

#define VAL_GPIOD_ASCR                  (PIN_ASCR_DISABLED(PD00) | \
					 PIN_ASCR_DISABLED(PD01) | \
					 PIN_ASCR_DISABLED(PD02) | \
					 PIN_ASCR_DISABLED(PD03) | \
					 PIN_ASCR_DISABLED(PD04) | \
					 PIN_ASCR_DISABLED(PD05) | \
					 PIN_ASCR_DISABLED(PD06) | \
					 PIN_ASCR_DISABLED(PD07) | \
					 PIN_ASCR_DISABLED(PD08) | \
					 PIN_ASCR_DISABLED(PD09) | \
					 PIN_ASCR_DISABLED(PD10) | \
					 PIN_ASCR_DISABLED(PD11) | \
					 PIN_ASCR_DISABLED(PD12) | \
					 PIN_ASCR_DISABLED(PD13) | \
					 PIN_ASCR_DISABLED(PD14) | \
					 PIN_ASCR_DISABLED(PD15))

#define VAL_GPIOD_LOCKR                 (PIN_LOCKR_DISABLED(PD00) | \
					 PIN_LOCKR_DISABLED(PD01) | \
					 PIN_LOCKR_DISABLED(PD02) | \
					 PIN_LOCKR_DISABLED(PD03) | \
					 PIN_LOCKR_DISABLED(PD04) | \
					 PIN_LOCKR_DISABLED(PD05) | \
					 PIN_LOCKR_DISABLED(PD06) | \
					 PIN_LOCKR_DISABLED(PD07) | \
					 PIN_LOCKR_DISABLED(PD08) | \
					 PIN_LOCKR_DISABLED(PD09) | \
					 PIN_LOCKR_DISABLED(PD10) | \
					 PIN_LOCKR_DISABLED(PD11) | \
					 PIN_LOCKR_DISABLED(PD12) | \
					 PIN_LOCKR_DISABLED(PD13) | \
					 PIN_LOCKR_DISABLED(PD14) | \
					 PIN_LOCKR_DISABLED(PD15))

#define VAL_GPIOH_MODER                 (PIN_MODE_ANALOG(PH00) | \
					 PIN_MODE_ANALOG(PH01) | \
					 PIN_MODE_ANALOG(PH02) | \
					 PIN_MODE_ANALOG(PH03) | \
					 PIN_MODE_ANALOG(PH04) | \
					 PIN_MODE_ANALOG(PH05) | \
					 PIN_MODE_ANALOG(PH06) | \
					 PIN_MODE_ANALOG(PH07) | \
					 PIN_MODE_ANALOG(PH08) | \
					 PIN_MODE_ANALOG(PH09) | \
					 PIN_MODE_ANALOG(PH10) | \
					 PIN_MODE_ANALOG(PH11) | \
					 PIN_MODE_ANALOG(PH12) | \
					 PIN_MODE_ANALOG(PH13) | \
					 PIN_MODE_ANALOG(PH14) | \
					 PIN_MODE_ANALOG(PH15))

#define VAL_GPIOH_OTYPER                (PIN_OTYPE_PUSHPULL(PH00) | \
					 PIN_OTYPE_PUSHPULL(PH01) | \
					 PIN_OTYPE_PUSHPULL(PH02) | \
					 PIN_OTYPE_PUSHPULL(PH03) | \
					 PIN_OTYPE_PUSHPULL(PH04) | \
					 PIN_OTYPE_PUSHPULL(PH05) | \
					 PIN_OTYPE_PUSHPULL(PH06) | \
					 PIN_OTYPE_PUSHPULL(PH07) | \
					 PIN_OTYPE_PUSHPULL(PH08) | \
					 PIN_OTYPE_PUSHPULL(PH09) | \
					 PIN_OTYPE_PUSHPULL(PH10) | \
					 PIN_OTYPE_PUSHPULL(PH11) | \
					 PIN_OTYPE_PUSHPULL(PH12) | \
					 PIN_OTYPE_PUSHPULL(PH13) | \
					 PIN_OTYPE_PUSHPULL(PH14) | \
					 PIN_OTYPE_PUSHPULL(PH15))

#define VAL_GPIOH_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PH00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PH15))

#define VAL_GPIOH_PUPDR                 (PIN_PUPDR_FLOATING(PH00) | \
					 PIN_PUPDR_FLOATING(PH01) | \
					 PIN_PUPDR_FLOATING(PH02) | \
					 PIN_PUPDR_FLOATING(PH03) | \
					 PIN_PUPDR_FLOATING(PH04) | \
					 PIN_PUPDR_FLOATING(PH05) | \
					 PIN_PUPDR_FLOATING(PH06) | \
					 PIN_PUPDR_FLOATING(PH07) | \
					 PIN_PUPDR_FLOATING(PH08) | \
					 PIN_PUPDR_FLOATING(PH09) | \
					 PIN_PUPDR_FLOATING(PH10) | \
					 PIN_PUPDR_FLOATING(PH11) | \
					 PIN_PUPDR_FLOATING(PH12) | \
					 PIN_PUPDR_FLOATING(PH13) | \
					 PIN_PUPDR_FLOATING(PH14) | \
					 PIN_PUPDR_FLOATING(PH15))

#define VAL_GPIOH_ODR                   (PIN_ODR_LEVEL_HIGH(PH00) | \
					 PIN_ODR_LEVEL_HIGH(PH01) | \
					 PIN_ODR_LEVEL_HIGH(PH02) | \
					 PIN_ODR_LEVEL_HIGH(PH03) | \
					 PIN_ODR_LEVEL_HIGH(PH04) | \
					 PIN_ODR_LEVEL_HIGH(PH05) | \
					 PIN_ODR_LEVEL_HIGH(PH06) | \
					 PIN_ODR_LEVEL_HIGH(PH07) | \
					 PIN_ODR_LEVEL_HIGH(PH08) | \
					 PIN_ODR_LEVEL_HIGH(PH09) | \
					 PIN_ODR_LEVEL_HIGH(PH10) | \
					 PIN_ODR_LEVEL_HIGH(PH11) | \
					 PIN_ODR_LEVEL_HIGH(PH12) | \
					 PIN_ODR_LEVEL_HIGH(PH13) | \
					 PIN_ODR_LEVEL_HIGH(PH14) | \
					 PIN_ODR_LEVEL_HIGH(PH15))

#define VAL_GPIOH_AFRL			(PIN_AFIO_AF(PH00, 0) | \
					 PIN_AFIO_AF(PH01, 0) | \
					 PIN_AFIO_AF(PH02, 0) | \
					 PIN_AFIO_AF(PH03, 0) | \
					 PIN_AFIO_AF(PH04, 0) | \
					 PIN_AFIO_AF(PH05, 0) | \
					 PIN_AFIO_AF(PH06, 0) | \
					 PIN_AFIO_AF(PH07, 0))

#define VAL_GPIOH_AFRH			(PIN_AFIO_AF(PH08, 0) | \
					 PIN_AFIO_AF(PH09, 0) | \
					 PIN_AFIO_AF(PH10, 0) | \
					 PIN_AFIO_AF(PH11, 0) | \
					 PIN_AFIO_AF(PH12, 0) | \
					 PIN_AFIO_AF(PH13, 0) | \
					 PIN_AFIO_AF(PH14, 0) | \
					 PIN_AFIO_AF(PH15, 0))

#define VAL_GPIOH_ASCR                  (PIN_ASCR_DISABLED(PH00) | \
					 PIN_ASCR_DISABLED(PH01) | \
					 PIN_ASCR_DISABLED(PH02) | \
					 PIN_ASCR_DISABLED(PH03) | \
					 PIN_ASCR_DISABLED(PH04) | \
					 PIN_ASCR_DISABLED(PH05) | \
					 PIN_ASCR_DISABLED(PH06) | \
					 PIN_ASCR_DISABLED(PH07) | \
					 PIN_ASCR_DISABLED(PH08) | \
					 PIN_ASCR_DISABLED(PH09) | \
					 PIN_ASCR_DISABLED(PH10) | \
					 PIN_ASCR_DISABLED(PH11) | \
					 PIN_ASCR_DISABLED(PH12) | \
					 PIN_ASCR_DISABLED(PH13) | \
					 PIN_ASCR_DISABLED(PH14) | \
					 PIN_ASCR_DISABLED(PH15))

#define VAL_GPIOH_LOCKR                 (PIN_LOCKR_DISABLED(PH00) | \
					 PIN_LOCKR_DISABLED(PH01) | \
					 PIN_LOCKR_DISABLED(PH02) | \
					 PIN_LOCKR_DISABLED(PH03) | \
					 PIN_LOCKR_DISABLED(PH04) | \
					 PIN_LOCKR_DISABLED(PH05) | \
					 PIN_LOCKR_DISABLED(PH06) | \
					 PIN_LOCKR_DISABLED(PH07) | \
					 PIN_LOCKR_DISABLED(PH08) | \
					 PIN_LOCKR_DISABLED(PH09) | \
					 PIN_LOCKR_DISABLED(PH10) | \
					 PIN_LOCKR_DISABLED(PH11) | \
					 PIN_LOCKR_DISABLED(PH12) | \
					 PIN_LOCKR_DISABLED(PH13) | \
					 PIN_LOCKR_DISABLED(PH14) | \
					 PIN_LOCKR_DISABLED(PH15))

#define VAL_GPIOE_MODER                 (PIN_MODE_ANALOG(PE00) | \
					 PIN_MODE_ANALOG(PE01) | \
					 PIN_MODE_ANALOG(PE02) | \
					 PIN_MODE_ANALOG(PE03) | \
					 PIN_MODE_ANALOG(PE04) | \
					 PIN_MODE_ANALOG(PE05) | \
					 PIN_MODE_ANALOG(PE06) | \
					 PIN_MODE_ANALOG(PE07) | \
					 PIN_MODE_ANALOG(PE08) | \
					 PIN_MODE_ANALOG(PE09) | \
					 PIN_MODE_ANALOG(PE10) | \
					 PIN_MODE_ANALOG(PE11) | \
					 PIN_MODE_ANALOG(PE12) | \
					 PIN_MODE_ANALOG(PE13) | \
					 PIN_MODE_ANALOG(PE14) | \
					 PIN_MODE_ANALOG(PE15))

#define VAL_GPIOE_OTYPER                (PIN_OTYPE_PUSHPULL(PE00) | \
					 PIN_OTYPE_PUSHPULL(PE01) | \
					 PIN_OTYPE_PUSHPULL(PE02) | \
					 PIN_OTYPE_PUSHPULL(PE03) | \
					 PIN_OTYPE_PUSHPULL(PE04) | \
					 PIN_OTYPE_PUSHPULL(PE05) | \
					 PIN_OTYPE_PUSHPULL(PE06) | \
					 PIN_OTYPE_PUSHPULL(PE07) | \
					 PIN_OTYPE_PUSHPULL(PE08) | \
					 PIN_OTYPE_PUSHPULL(PE09) | \
					 PIN_OTYPE_PUSHPULL(PE10) | \
					 PIN_OTYPE_PUSHPULL(PE11) | \
					 PIN_OTYPE_PUSHPULL(PE12) | \
					 PIN_OTYPE_PUSHPULL(PE13) | \
					 PIN_OTYPE_PUSHPULL(PE14) | \
					 PIN_OTYPE_PUSHPULL(PE15))

#define VAL_GPIOE_OSPEEDR               (PIN_OSPEED_SPEED_VERYLOW(PE00) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE01) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE02) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE03) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE04) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE05) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE06) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE07) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE08) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE09) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE10) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE11) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE12) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE13) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE14) | \
					 PIN_OSPEED_SPEED_VERYLOW(PE15))

#define VAL_GPIOE_PUPDR                 (PIN_PUPDR_FLOATING(PE00) | \
					 PIN_PUPDR_FLOATING(PE01) | \
					 PIN_PUPDR_FLOATING(PE02) | \
					 PIN_PUPDR_FLOATING(PE03) | \
					 PIN_PUPDR_FLOATING(PE04) | \
					 PIN_PUPDR_FLOATING(PE05) | \
					 PIN_PUPDR_FLOATING(PE06) | \
					 PIN_PUPDR_FLOATING(PE07) | \
					 PIN_PUPDR_FLOATING(PE08) | \
					 PIN_PUPDR_FLOATING(PE09) | \
					 PIN_PUPDR_FLOATING(PE10) | \
					 PIN_PUPDR_FLOATING(PE11) | \
					 PIN_PUPDR_FLOATING(PE12) | \
					 PIN_PUPDR_FLOATING(PE13) | \
					 PIN_PUPDR_FLOATING(PE14) | \
					 PIN_PUPDR_FLOATING(PE15))

#define VAL_GPIOE_ODR                   (PIN_ODR_LEVEL_HIGH(PE00) | \
					 PIN_ODR_LEVEL_HIGH(PE01) | \
					 PIN_ODR_LEVEL_HIGH(PE02) | \
					 PIN_ODR_LEVEL_HIGH(PE03) | \
					 PIN_ODR_LEVEL_HIGH(PE04) | \
					 PIN_ODR_LEVEL_HIGH(PE05) | \
					 PIN_ODR_LEVEL_HIGH(PE06) | \
					 PIN_ODR_LEVEL_HIGH(PE07) | \
					 PIN_ODR_LEVEL_HIGH(PE08) | \
					 PIN_ODR_LEVEL_HIGH(PE09) | \
					 PIN_ODR_LEVEL_HIGH(PE10) | \
					 PIN_ODR_LEVEL_HIGH(PE11) | \
					 PIN_ODR_LEVEL_HIGH(PE12) | \
					 PIN_ODR_LEVEL_HIGH(PE13) | \
					 PIN_ODR_LEVEL_HIGH(PE14) | \
					 PIN_ODR_LEVEL_HIGH(PE15))

#define VAL_GPIOE_AFRL			(PIN_AFIO_AF(PE00, 0) | \
					 PIN_AFIO_AF(PE01, 0) | \
					 PIN_AFIO_AF(PE02, 0) | \
					 PIN_AFIO_AF(PE03, 0) | \
					 PIN_AFIO_AF(PE04, 0) | \
					 PIN_AFIO_AF(PE05, 0) | \
					 PIN_AFIO_AF(PE06, 0) | \
					 PIN_AFIO_AF(PE07, 0))

#define VAL_GPIOE_AFRH			(PIN_AFIO_AF(PE08, 0) | \
					 PIN_AFIO_AF(PE09, 0) | \
					 PIN_AFIO_AF(PE10, 0) | \
					 PIN_AFIO_AF(PE11, 0) | \
					 PIN_AFIO_AF(PE12, 0) | \
					 PIN_AFIO_AF(PE13, 0) | \
					 PIN_AFIO_AF(PE14, 0) | \
					 PIN_AFIO_AF(PE15, 0))

#define VAL_GPIOE_ASCR                  (PIN_ASCR_DISABLED(PE00) | \
					 PIN_ASCR_DISABLED(PE01) | \
					 PIN_ASCR_DISABLED(PE02) | \
					 PIN_ASCR_DISABLED(PE03) | \
					 PIN_ASCR_DISABLED(PE04) | \
					 PIN_ASCR_DISABLED(PE05) | \
					 PIN_ASCR_DISABLED(PE06) | \
					 PIN_ASCR_DISABLED(PE07) | \
					 PIN_ASCR_DISABLED(PE08) | \
					 PIN_ASCR_DISABLED(PE09) | \
					 PIN_ASCR_DISABLED(PE10) | \
					 PIN_ASCR_DISABLED(PE11) | \
					 PIN_ASCR_DISABLED(PE12) | \
					 PIN_ASCR_DISABLED(PE13) | \
					 PIN_ASCR_DISABLED(PE14) | \
					 PIN_ASCR_DISABLED(PE15))

#define VAL_GPIOE_LOCKR                 (PIN_LOCKR_DISABLED(PE00) | \
					 PIN_LOCKR_DISABLED(PE01) | \
					 PIN_LOCKR_DISABLED(PE02) | \
					 PIN_LOCKR_DISABLED(PE03) | \
					 PIN_LOCKR_DISABLED(PE04) | \
					 PIN_LOCKR_DISABLED(PE05) | \
					 PIN_LOCKR_DISABLED(PE06) | \
					 PIN_LOCKR_DISABLED(PE07) | \
					 PIN_LOCKR_DISABLED(PE08) | \
					 PIN_LOCKR_DISABLED(PE09) | \
					 PIN_LOCKR_DISABLED(PE10) | \
					 PIN_LOCKR_DISABLED(PE11) | \
					 PIN_LOCKR_DISABLED(PE12) | \
					 PIN_LOCKR_DISABLED(PE13) | \
					 PIN_LOCKR_DISABLED(PE14) | \
					 PIN_LOCKR_DISABLED(PE15))

#define AF_PA00_RCC_CK_IN                0U
#define AF_LINE_RCC_CK_IN                0U
#define AF_PA02_SHELL_TX                 7U
#define AF_LINE_SHELL_TX                 7U
#define AF_PA03_CASTEL_ICU2              14U
#define AF_LINE_CASTEL_ICU2              14U
#define AF_PA05_CASTEL_ICU1              1U
#define AF_LINE_CASTEL_ICU1              1U
#define AF_PA08_CASTEL_PWM1              1U
#define AF_LINE_CASTEL_PWM1              1U
#define AF_PA11_CASTEL_PWM2              1U
#define AF_LINE_CASTEL_PWM2              1U
#define AF_PA13_SWDIO                    0U
#define AF_LINE_SWDIO                    0U
#define AF_PA14_SWCLK                    0U
#define AF_LINE_SWCLK                    0U
#define AF_PA15_SHELL_RX                 3U
#define AF_LINE_SHELL_RX                 3U
#define AF_PB06_CASTEL_TX                7U
#define AF_LINE_CASTEL_TX                7U
#define AF_PB07_CASTEL_RX                7U
#define AF_LINE_CASTEL_RX                7U
#define AF_PC14_RCC_OSC32_IN             0U
#define AF_LINE_RCC_OSC32_IN             0U
#define AF_PC15_RCC_OSC32_OUT            0U
#define AF_LINE_RCC_OSC32_OUT            0U


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

