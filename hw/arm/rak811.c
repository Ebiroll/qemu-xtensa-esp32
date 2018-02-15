/*-
 * Copyright (c) 2013, 2018
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <inttypes.h>
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "hw/ssi/ssi.h"
#include "hw/boards.h"
#include "hw/block/flash.h"
#include "sysemu/sysemu.h"
#include "sysemu/blockdev.h"
#include "ui/console.h"
#include "rak811.h"
#include "stm32l1xx.h"


//#define DEBUG_RAK811
#ifdef DEBUG_RAK811
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                       \
    do { printf("DEBUG_PEBBLE: " fmt , ## __VA_ARGS__); \
         usleep(1000); \
    } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

const PblBoardConfig s_board_config_rak811 = {
    .dbgserial_uart_index = 2,       // USART3
    .pebble_control_uart_index = 1,  // USART2
    .button_map = {
        {STM32_GPIOA_INDEX, 1}   // boot
    },
    .flash_size = 4096,  /* Kbytes - larger to aid in development and debugging */
    .ram_size = 32  /* Kbytes */
};


// ----------------------------------------------------------------------------------------
// Static globals
static PblButtonID s_waiting_key_up_id = PBL_BUTTON_ID_NONE;
static QEMUTimer *s_button_timer;


// The irq callbacks for each button
static qemu_irq s_button_irq[PBL_NUM_BUTTONS];
static qemu_irq s_button_wakeup;


static void prv_send_key_up(void *opaque)
{
    qemu_irq *button_irqs = opaque;
    if (s_waiting_key_up_id == PBL_BUTTON_ID_NONE) {
        /* Should never happen */
        return;
    }

    DPRINTF("button %d released\n", s_waiting_key_up_id);
    qemu_set_irq(button_irqs[s_waiting_key_up_id], true);
    qemu_set_irq(s_button_wakeup, false);
    s_waiting_key_up_id = PBL_BUTTON_ID_NONE;
}


// NOTE: When running using a VNC display, we alwqys get a key-up immediately after the key-down,
// even if the user is holding the key down. For long presses, this results in a series of
// quick back to back key-down, key-up callbacks.
static void rak811_key_handler(void *arg, int keycode)
{
    qemu_irq *button_irqs = arg;
    static int prev_keycode;

    int pressed = (keycode & 0x80) == 0;
    int button_id = PBL_BUTTON_ID_NONE;

    switch (keycode & 0x7F) {
    case 16: /* Q */
        //button_id = PBL_BUTTON_ID_BACK;
        break;
    case 17: /* W */
        //button_id = PBL_BUTTON_ID_UP;
        break;
    case 31: /* S */
        //button_id = PBL_BUTTON_ID_SELECT;
        break;
    case 45: /* X */
        //button_id = PBL_BUTTON_ID_DOWN;
        break;
    case 72: /* up arrow */
        if (prev_keycode == 224) {
            //button_id = PBL_BUTTON_ID_UP;
        }
        break;
    case 80: /* down arrow */
        if (prev_keycode == 224) {
           // button_id = PBL_BUTTON_ID_DOWN;
        }
        break;
    case 75: /* left arrow */
        if (prev_keycode == 224) {
            //button_id = PBL_BUTTON_ID_BACK;
        }
        break;
    case 77: /* right arrow */
        if (prev_keycode == 224) {
            //button_id = PBL_BUTTON_ID_SELECT;
        }
        break;
    default:
        break;
    }

    prev_keycode = keycode;
    if (button_id == PBL_BUTTON_ID_NONE || !pressed) {
        /* Ignore key ups and keys we don't care about */
        return;
    }

    // If this is a different key, and we are waiting for the prior one to key up, send the
    //  key up now
    if (s_waiting_key_up_id != PBL_BUTTON_ID_NONE && button_id != s_waiting_key_up_id) {
        prv_send_key_up(button_irqs);
    }

    if (s_waiting_key_up_id != button_id) {
        DPRINTF("button %d pressed\n", button_id);
        s_waiting_key_up_id = button_id;
        qemu_set_irq(button_irqs[button_id], false);   // Pressed
        qemu_set_irq(s_button_wakeup, true);
    }

    /* Set or reschedule the timer to release the key */
    if (!s_button_timer) {
        s_button_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, prv_send_key_up, button_irqs);
    }
    timer_mod(s_button_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 250);
}


// ------------------------------------------------------------------------------------------
// This method used externally (by pebble_control) for setting a given button state
void rak811_set_button_state(uint32_t button_state)
{
    // Toggle the GPIOs to match the new button state
    int button_id;
    for (button_id=0; button_id < PBL_NUM_BUTTONS; button_id++) {
      uint32_t mask = 1 << button_id;
      qemu_set_irq(s_button_irq[button_id], !(button_state & mask));   // Set new state
    }
}


// -----------------------------------------------------------------------------------------
// Init button handling
void rak811_init_buttons(Stm32Gpio *gpio[], const PblButtonMap *map)
{
    int i;
    for (i = 0; i < PBL_NUM_BUTTONS; i++) {
        qemu_irq irq = qdev_get_gpio_in((DeviceState *)gpio[map[i].gpio], map[i].pin);
        if (map[i].active_high) {
            s_button_irq[i] = qemu_irq_invert(irq);

        } else {
            s_button_irq[i] = irq;
        }
    }
    // GPIO A, pin 0 is the WKUP pin.
    s_button_wakeup = qdev_get_gpio_in((DeviceState *)gpio[STM32_GPIOA_INDEX], 0);
    qemu_add_kbd_event_handler(rak811_key_handler, s_button_irq);
}


// ----------------------------------------------------------------------------------------
// Init the board device
DeviceState *rak811_init_board(Stm32Gpio *gpio[], qemu_irq display_vibe)
{
    // Create the board device and wire it up
    DeviceState *board = qdev_create(NULL, "rak811_board");
    qdev_prop_set_ptr(board, "name", (void *)"Pebble");

#ifndef rak811_NO_DISPLAY_VIBRATE
    qdev_prop_set_ptr(board, "display_vibe", display_vibe);
#endif

    qdev_init_nofail(board);
    return board;
}

// ----------------------------------------------------------------------------------------
// Set our QEMU specific settings to the target
void rak811_set_qemu_settings(DeviceState *rtc_dev)
{
    #define QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE  0x00000001
    #define QEMU_REG_0_START_CONNECTED          0x00000002
    #define QEMU_REG_0_START_PLUGGED_IN         0x00000004

    // Default settings
    uint32_t  flags = QEMU_REG_0_START_CONNECTED;

    // Set the QEMU specific settings in the extra backup registers
    char *strval;
    strval = getenv("rak811_QEMU_FIRST_BOOT_LOGIC_ENABLE");
    if (strval) {
        // If set, allow "first boot" behavior, which displays the "Ready for Update"
        // screen
        if (atoi(strval)) {
            flags |= QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE;
        } else {
            flags &= ~QEMU_REG_0_FIRST_BOOT_LOGIC_ENABLE;
        }
    }

    strval = getenv("rak811_QEMU_START_CONNECTED");
    if (strval) {
        // If set, default to bluetooth not connected
        if (atoi(strval)) {
            flags |= QEMU_REG_0_START_CONNECTED;
        } else {
            flags &= ~QEMU_REG_0_START_CONNECTED;
        }

    }

    strval = getenv("rak811_QEMU_START_PLUGGED_IN");
    if (strval) {
        // If set, default to plugged in
        if (atoi(strval)) {
            flags |= QEMU_REG_0_START_PLUGGED_IN;
        } else {
            flags &= ~QEMU_REG_0_START_PLUGGED_IN;
        }
    }

    f2xx_rtc_set_extra_bkup_reg(rtc_dev, 0, flags);
}



// ------------------------------------------------------------------------------------------
// Instantiate a 32l1xx based pebble
static void rak811_32l1_init(MachineState *machine, const PblBoardConfig *board_config)
{
    Stm32Gpio *gpio[STM32F2XX_GPIO_COUNT];
    Stm32Uart *uart[STM32F2XX_UART_COUNT];
    Stm32Timer *timer[STM32F2XX_TIM_COUNT];
    DeviceState *spi_flash;
    DeviceState *rtc_dev;
    SSIBus *spi;
    stm32l1xx_t stm;
    ARMCPU *cpu;

    // Note: allow for bigger flash images (4MByte) to aid in development and debugging
    stm32l1xx_init(
        board_config->flash_size,
        board_config->ram_size,
        machine->kernel_filename,
        gpio,
        uart,
        timer,
        &rtc_dev,
        8000000, /* osc_freq*/
        32768, /* osc2_freq*/
        &stm,
        &cpu);


    // Set the Pebble specific QEMU settings on the target
    rak811_set_qemu_settings(rtc_dev);

    /* SPI flash */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[0], "ssi");
    spi_flash = ssi_create_slave_no_init(spi, "n25q032a11");
    qdev_init_nofail(spi_flash);

    qemu_irq cs;
    cs = qdev_get_gpio_in_named(spi_flash, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOA_INDEX], 4, cs);


    /* Display */
    spi = (SSIBus *)qdev_get_child_bus(stm.spi_dev[1], "ssi");
    DeviceState *display_dev = ssi_create_slave_no_init(spi, "sm-lcd");
    qdev_prop_set_bit(display_dev, "rotate_display", true);
    qdev_init_nofail(display_dev);

    qemu_irq backlight_enable;
    backlight_enable = qdev_get_gpio_in_named(display_dev, "backlight_enable", 0);
    qdev_connect_gpio_out_named((DeviceState *)gpio[STM32_GPIOB_INDEX], "af", 5,
                                  backlight_enable);

    qemu_irq backlight_level;
    backlight_level = qdev_get_gpio_in_named(display_dev, "backlight_level", 0);
    qdev_connect_gpio_out_named((DeviceState *)timer[2], "pwm_ratio_changed", 0, // TIM3
                                  backlight_level);

    // TODO!!!!!!
    qemu_irq display_power;
    display_power = qdev_get_gpio_in_named(display_dev, "power_ctl", 0);
    //qdev_connect_gpio_out_named((DeviceState *)cpu->env.nvic, "power_out", 0,
    //                              display_power);

    // Connect up the uarts
    //rak811_connect_uarts(uart, board_config);

    // Init the buttons
    rak811_init_buttons(gpio, board_config->button_map);

    // Create the board device and wire it up
    qemu_irq display_vibe;
    display_vibe = qdev_get_gpio_in_named(display_dev, "vibe_ctl", 0);
    DeviceState *board = rak811_init_board(gpio, display_vibe);

    // The GPIO from vibrate drives the vibe input on the board
    qemu_irq board_vibe_in;
    board_vibe_in = qdev_get_gpio_in_named(board, "rak811_board_vibe_in", 0);
    qdev_connect_gpio_out((DeviceState *)gpio[STM32_GPIOB_INDEX], 0, board_vibe_in);
}



// ================================================================================
// Pebble "board" device. Used when we need to fan out a GPIO outputs to one or more other
// devices/instances
typedef struct PebbleBoard {
    SysBusDevice busdev;
    void *name;

    union {
      void *vibe_out_irq_prop;
      qemu_irq vibe_out_irq;
    };

} PebbleBoard;


static void rak811_board_vibe_ctl(void *opaque, int n, int level)
{
    PebbleBoard *s = (PebbleBoard *)opaque;
    assert(n == 0);

    // Tell the pebble control instance that we vibrated
    //rak811_control_send_vibe_notification(s_rak811_control, level != 0);

    // Tell pass onto the vibe out output as well
    qemu_set_irq(s->vibe_out_irq, level);
}


static int rak811_board_init(SysBusDevice *dev)
{
    //PebbleBoard *s = FROM_SYSBUS(PebbleBoard, dev);

    /* This callback informs us that the vibrate is on/orr */
    qdev_init_gpio_in_named(DEVICE(dev), rak811_board_vibe_ctl,
                            "rak811_board_vibe_in", 1);

    return 0;
}



static Property rak811_board_properties[] = {
    DEFINE_PROP_PTR("name", PebbleBoard, name),
    DEFINE_PROP_PTR("display_vibe", PebbleBoard, vibe_out_irq_prop),
    DEFINE_PROP_END_OF_LIST(),
};

static void rak811_board_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *sc = SYS_BUS_DEVICE_CLASS(klass);
    sc->init = rak811_board_init;
    dc->props = rak811_board_properties;
}

static const TypeInfo rak811_board_info = {
    .name          = "rak811",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PebbleBoard),
    .class_init    = rak811_board_class_init,
};

static void rak811_board_register_types(void)
{
    type_register_static(&rak811_board_info);
}

type_init(rak811_board_register_types)


// ================================================================================
// Machines
static void rak811_bb2_init(MachineState *machine)
{
    rak811_32l1_init(machine, &s_board_config_rak811);
}


static void rak811_bb2_machine_init(MachineClass *mc)
{
    mc->desc = "Pebble smartwatch (bb2/ev1/ev2)";
    mc->init = rak811_bb2_init;
}

DEFINE_MACHINE("rak811", rak811_bb2_machine_init)

/*
static void rak811_bb_machine_init(MachineClass *mc)
{
  mc->desc = "RAK 811 Lora Wan";
  mc->init = rak811_bb2_init;
}

DEFINE_MACHINE("rak811", rak811_bb_machine_init)

*/