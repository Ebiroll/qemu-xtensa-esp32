#ifndef STM32L1XX_
#define STM32L1XX_

#include "hw/arm/stm32.h"

#define STM32L1XX_GPIO_COUNT (STM32_GPIOI - STM32_GPIOA + 1)
#define STM32L1XX_SPI_COUNT 3

#define STM32L1XX_UART_COUNT 6
#define STM32L1XX_TIM_COUNT 14

struct stm32l1XX {
    DeviceState *spi_dev[STM32L1XX_SPI_COUNT];
};

#endif