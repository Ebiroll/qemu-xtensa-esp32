/*
 * Olimex STM32 P103 Development Board
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on
 * Olimex "STM-P103 Development Board Users Manual Rev. A, April 2008"
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
#include "stm32f1xx.h"
#include "hw/sysbus.h"
#include "hw/arm/arm.h"
#include "hw/devices.h"
#include "ui/console.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"


typedef struct {
    Stm32 *stm32;
    Stm32Gpio *stm32_gpio[STM32F1XX_GPIO_COUNT];
    Stm32Uart *stm32_uart[STM32_UART_COUNT];

    bool last_button_pressed;
    qemu_irq button_irq;
} Stm32P103;




static void led_irq_handler(void *opaque, int n, int level)
{
    /* There should only be one IRQ for the LED */
    assert(n == 0);

    /* Assume that the IRQ is only triggered if the LED has changed state.
     * If this is not correct, we may get multiple LED Offs or Ons in a row.
     */
    switch (level) {
        case 0:
            printf("LED Off\n");
            break;
        case 1:
            printf("LED On\n");
            break;
    }
}

static void stm32_p103_key_event(void *opaque, int keycode)
{
    Stm32P103 *s = (Stm32P103 *)opaque;
    bool make;
    int core_keycode;

    if((keycode & 0x80) == 0) {
        make = true;
        core_keycode = keycode;
    } else {
        make = false;
        core_keycode = keycode & 0x7f;
    }

    /* Responds when a "B" key press is received.
     * Inside the monitor, you can type "sendkey b"
     */
    if(core_keycode == 0x30) {
        if(make) {
            if(!s->last_button_pressed) {
                qemu_irq_raise(s->button_irq);
                s->last_button_pressed = true;
            }
        } else {
            if(s->last_button_pressed) {
                qemu_irq_lower(s->button_irq);
                s->last_button_pressed = false;
            }
        }
    }
    return;

}


static void stm32_p103_init(MachineState *machine) {
    
    const char* kernel_filename = machine->kernel_filename;
    qemu_irq *led_irq;
    Stm32P103 *s;

    s = (Stm32P103 *)g_malloc0(sizeof(Stm32P103));

    stm32f1xx_init(/*flash_size in bytes */ 128 * 1024,
               /*ram_size in bytes */ 32 * 1024,
               kernel_filename,
               s->stm32_gpio,
               s->stm32_uart,
               8000000,
               32768);

    /* Connect LED to GPIO C pin 12 */
    led_irq = qemu_allocate_irqs(led_irq_handler, NULL, 1);
    qdev_connect_gpio_out((DeviceState *)s->stm32_gpio[STM32_GPIOC_INDEX], 12, led_irq[0]);

    /* Connect button to GPIO A pin 0 */
    s->button_irq = qdev_get_gpio_in((DeviceState *)s->stm32_gpio[STM32_GPIOA_INDEX], 0);
    qemu_add_kbd_event_handler(stm32_p103_key_event, s);

    /* Connect RS232 to UART */
    stm32_uart_connect(
            s->stm32_uart[STM32_UART2_INDEX],
            serial_hds[0],
            STM32_USART2_NO_REMAP);
}

static void stm32_p103_machine_init(MachineClass *mc)
{
    mc->desc = "Olimex STM32 p103 Dev Board";
    mc->init = stm32_p103_init;
}

DEFINE_MACHINE("stm32-p103", stm32_p103_machine_init)
