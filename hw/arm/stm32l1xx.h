#ifndef STM32L1XX_
#define STM32L1XX_

#include "hw/arm/stm32.h"
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "sysemu/sysemu.h"
#include "qapi/visitor.h"
#include "qemu/error-report.h"
#include "hw/hw.h"
#include "hw/qdev-core.h"

#define STM32L1XX_GPIO_COUNT (STM32_GPIOI - STM32_GPIOA + 1)
#define STM32L1XX_SPI_COUNT 3

#define STM32L1XX_UART_COUNT 3
#define STM32L1XX_TIM_COUNT 12

typedef struct stm32l1XX {
    DeviceState *spi_dev[STM32L1XX_SPI_COUNT];
} stm32l1xx_t;

#endif