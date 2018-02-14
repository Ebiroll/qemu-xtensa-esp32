/*
 * STM32 Microcontroller
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include <stdio.h>
#include <inttypes.h>
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/arm/stm32.h"
#include "stm32f1xx.h"
#include "exec/address-spaces.h"

/* PERIPHERALS */

static const char *stm32f1xx_periph_name_arr[] = {
    ENUM_STRING(STM32F1XX_RCC),
    ENUM_STRING(STM32F1XX_GPIOA),
    ENUM_STRING(STM32F1XX_GPIOB),
    ENUM_STRING(STM32F1XX_GPIOC),
    ENUM_STRING(STM32F1XX_GPIOD),
    ENUM_STRING(STM32F1XX_GPIOE),
    ENUM_STRING(STM32F1XX_GPIOF),
    ENUM_STRING(STM32F1XX_GPIOG),
    ENUM_STRING(STM32F1XX_AFIO),
    ENUM_STRING(STM32F1XX_UART1),
    ENUM_STRING(STM32F1XX_UART2),
    ENUM_STRING(STM32F1XX_UART3),
    ENUM_STRING(STM32F1XX_UART4),
    ENUM_STRING(STM32F1XX_UART5),
    ENUM_STRING(STM32F1XX_ADC1),
    ENUM_STRING(STM32F1XX_ADC2),
    ENUM_STRING(STM32F1XX_ADC3),
    ENUM_STRING(STM32F1XX_DAC),
    ENUM_STRING(STM32F1XX_TIM1),
    ENUM_STRING(STM32F1XX_TIM2),
    ENUM_STRING(STM32F1XX_TIM3),
    ENUM_STRING(STM32F1XX_TIM4),
    ENUM_STRING(STM32F1XX_TIM5),
    ENUM_STRING(STM32F1XX_TIM6),
    ENUM_STRING(STM32F1XX_TIM7),
    ENUM_STRING(STM32F1XX_TIM8),
    ENUM_STRING(STM32F1XX_BKP),
    ENUM_STRING(STM32F1XX_PWR),
    ENUM_STRING(STM32F1XX_I2C1),
    ENUM_STRING(STM32F1XX_I2C2),
    ENUM_STRING(STM32F1XX_I2S2),
    ENUM_STRING(STM32F1XX_I2S3),
    ENUM_STRING(STM32F1XX_WWDG),
    ENUM_STRING(STM32F1XX_CAN1),
    ENUM_STRING(STM32F1XX_CAN2),
    ENUM_STRING(STM32F1XX_CAN),
    ENUM_STRING(STM32F1XX_USB),
    ENUM_STRING(STM32F1XX_SPI1),
    ENUM_STRING(STM32F1XX_SPI2),
    ENUM_STRING(STM32F1XX_SPI3),
    ENUM_STRING(STM32F1XX_EXTI),
    ENUM_STRING(STM32F1XX_SDIO),
    ENUM_STRING(STM32F1XX_FSMC),
    ENUM_STRING(STM32F1XX_PERIPH_COUNT),
};

void stm32f1xx_init(
            ram_addr_t flash_size,
            ram_addr_t sram_size,
            const char *kernel_filename,
            Stm32Gpio *stm32_gpio[],
            Stm32Uart *stm32_uart[],
            uint32_t osc_freq,
            uint32_t osc32_freq)
{
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *flash_alias_mem = g_malloc(sizeof(MemoryRegion));
    DeviceState *nvic;
    int i;

    Object *stm32_container = container_get(qdev_get_machine(), "/stm32");

    //nvic = armv7m_init(system_memory, flash_size, NUM_IRQ_LINES,
    //                  kernel_filename, cpu_model);


    nvic = armv7m_init(
              //stm32_container,
              address_space_mem,
              flash_size,
              //sram_size,
              0, /* default number of irqs */
              kernel_filename,
              "cortex-m3");

    /* The STM32 family stores its Flash memory at some base address in memory
     * (0x08000000 for medium density devices), and then aliases it to the
     * boot memory space, which starts at 0x00000000 (the "System Memory" can also
     * be aliased to 0x00000000, but this is not implemented here). The processor
     * executes the code in the aliased memory at 0x00000000.  We need to make a
     * QEMU alias so that reads in the 0x08000000 area are passed through to the
     * 0x00000000 area. Note that this is the opposite of real hardware, where the
     * memory at 0x00000000 passes reads through the "real" flash memory at
     * 0x08000000, but it works the same either way. */
    /* TODO: Parameterize the base address of the aliased memory. */
    memory_region_init_alias(
            flash_alias_mem,
            NULL,
            "stm32-flash-alias-mem",
            address_space_mem,
            0,
            flash_size);
    memory_region_add_subregion(address_space_mem, 0x08000000, flash_alias_mem);

    DeviceState *rcc_dev = qdev_create(NULL, "stm32f1xx_rcc");
    qdev_prop_set_uint32(rcc_dev, "osc_freq", osc_freq);
    qdev_prop_set_uint32(rcc_dev, "osc32_freq", osc32_freq);
    object_property_add_child(stm32_container, "stm32f1xx_rcc", OBJECT(rcc_dev), NULL);
    stm32_init_periph(rcc_dev, STM32_RCC_PERIPH, 0x40021000, qdev_get_gpio_in(nvic, STM32_RCC_IRQ));

    DeviceState **gpio_dev = (DeviceState **)stm32_gpio;
    for(i = 0; i < STM32F1XX_GPIO_COUNT; i++) {
        char child_name[8];
        stm32_periph_t periph = STM32_GPIOA + i;
        gpio_dev[i] = qdev_create(NULL, TYPE_STM32_GPIO);
        gpio_dev[i]->id = stm32f1xx_periph_name_arr[periph];
        QDEV_PROP_SET_PERIPH_T(gpio_dev[i], "periph", periph);
        qdev_prop_set_ptr(gpio_dev[i], "stm32_rcc", rcc_dev);
        snprintf(child_name, sizeof(child_name), "gpio[%c]", 'a' + i);
        object_property_add_child(stm32_container, child_name, OBJECT(gpio_dev[i]), NULL);
        stm32_init_periph(gpio_dev[i], periph, 0x40010800 + (i * 0x400), NULL);
    }

    DeviceState *exti_dev = qdev_create(NULL, TYPE_STM32_EXTI);
    object_property_add_child(stm32_container, "exti", OBJECT(exti_dev), NULL);
    stm32_init_periph(exti_dev, STM32_EXTI_PERIPH, 0x40010400, NULL);
    SysBusDevice *exti_busdev = SYS_BUS_DEVICE(exti_dev);
    sysbus_connect_irq(exti_busdev, 0, qdev_get_gpio_in(nvic, STM32_EXTI0_IRQ));
    sysbus_connect_irq(exti_busdev, 1, qdev_get_gpio_in(nvic, STM32_EXTI1_IRQ));
    sysbus_connect_irq(exti_busdev, 2, qdev_get_gpio_in(nvic, STM32_EXTI2_IRQ));
    sysbus_connect_irq(exti_busdev, 3, qdev_get_gpio_in(nvic, STM32_EXTI3_IRQ));
    sysbus_connect_irq(exti_busdev, 4, qdev_get_gpio_in(nvic, STM32_EXTI4_IRQ));
    sysbus_connect_irq(exti_busdev, 5, qdev_get_gpio_in(nvic, STM32_EXTI9_5_IRQ));
    sysbus_connect_irq(exti_busdev, 6, qdev_get_gpio_in(nvic, STM32_EXTI15_10_IRQ));
    sysbus_connect_irq(exti_busdev, 7, qdev_get_gpio_in(nvic, STM32_PVD_IRQ));
    sysbus_connect_irq(exti_busdev, 8, qdev_get_gpio_in(nvic, STM32_RTCAlarm_IRQ));
    sysbus_connect_irq(exti_busdev, 9, qdev_get_gpio_in(nvic, STM32_OTG_FS_WKUP_IRQ));

    DeviceState *afio_dev = qdev_create(NULL, TYPE_STM32_AFIO);
    qdev_prop_set_ptr(afio_dev, "stm32_rcc", rcc_dev);
    object_property_set_link(OBJECT(afio_dev), OBJECT(gpio_dev[0]), "gpio[a]", NULL);
    object_property_set_link(OBJECT(afio_dev), OBJECT(gpio_dev[1]), "gpio[b]", NULL);
    object_property_set_link(OBJECT(afio_dev), OBJECT(gpio_dev[2]), "gpio[c]", NULL);
    object_property_set_link(OBJECT(afio_dev), OBJECT(gpio_dev[3]), "gpio[d]", NULL);
    object_property_set_link(OBJECT(afio_dev), OBJECT(gpio_dev[4]), "gpio[e]", NULL);
    object_property_set_link(OBJECT(afio_dev), OBJECT(gpio_dev[5]), "gpio[f]", NULL);
    object_property_set_link(OBJECT(afio_dev), OBJECT(gpio_dev[6]), "gpio[g]", NULL);
    object_property_set_link(OBJECT(afio_dev), OBJECT(exti_dev), "exti", NULL);
    object_property_add_child(stm32_container, "afio", OBJECT(afio_dev), NULL);
    stm32_init_periph(afio_dev, STM32_AFIO_PERIPH, 0x40010000, NULL);

    // Create UARTs:
    struct {
        uint32_t addr;
        uint8_t irq_idx;
    } const uart_desc[] = {
        {0x40013800, STM32_UART1_IRQ},
        {0x40004400, STM32_UART2_IRQ},
        {0x40004800, STM32_UART3_IRQ},
        {0x40004c00, STM32_UART4_IRQ},
        {0x40005000, STM32_UART5_IRQ},
    };
    for (i = 0; i < ARRAY_LENGTH(uart_desc); ++i) {
        const stm32_periph_t periph = STM32F1XX_UART1 + i;
        DeviceState *uart_dev = qdev_create(NULL, "stm32-uart");
        uart_dev->id = stm32f1xx_periph_name_arr[periph];
        qdev_prop_set_int32(uart_dev, "periph", periph);
        qdev_prop_set_ptr(uart_dev, "stm32_rcc", rcc_dev);
        qdev_prop_set_ptr(uart_dev, "stm32_gpio", gpio_dev);
        qdev_prop_set_ptr(uart_dev, "stm32_afio", afio_dev);
        qdev_prop_set_ptr(uart_dev, "stm32_check_tx_pin_callback", (void *)stm32_afio_uart_check_tx_pin_callback);
        stm32_init_periph(uart_dev, periph, uart_desc[i].addr,
                          qdev_get_gpio_in(nvic, uart_desc[i].irq_idx));
        stm32_uart[i] = (Stm32Uart*)uart_dev;
    }
    /*
    stm32_create_uart_dev(stm32_container, STM32_UART1, 1, rcc_dev, gpio_dev, afio_dev, 0x40013800, pic[STM32_UART1_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART2, 2, rcc_dev, gpio_dev, afio_dev, 0x40004400, pic[STM32_UART2_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART3, 3, rcc_dev, gpio_dev, afio_dev, 0x40004800, pic[STM32_UART3_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART4, 4, rcc_dev, gpio_dev, afio_dev, 0x40004c00, pic[STM32_UART4_IRQ]);
    stm32_create_uart_dev(stm32_container, STM32_UART5, 5, rcc_dev, gpio_dev, afio_dev, 0x40005000, pic[STM32_UART5_IRQ]);
    */
}
