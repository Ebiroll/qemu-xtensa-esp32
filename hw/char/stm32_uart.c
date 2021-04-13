/*
 * STM32 Microcontroller UART module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on pl011.c
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
#include "hw/sysbus.h"
#include "hw/arm/stm32.h"
#include "hw/arm/stm32f1xx.h"
#include "sysemu/char.h"
#include "qemu/bitops.h"



/* DEFINITIONS*/

/* See the README file for details on these settings. */
//#define DEBUG_STM32_UART
//#define STM32_UART_NO_BAUD_DELAY
//#define STM32_UART_ENABLE_OVERRUN

#ifdef DEBUG_STM32_UART
#define DPRINTF(fmt, ...)                                       \
    do { printf("STM32_UART: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define USART_SR_OFFSET 0x00
#define USART_SR_TXE_BIT 7
#define USART_SR_TC_BIT 6
#define USART_SR_RXNE_BIT 5
#define USART_SR_ORE_BIT 3

#define USART_DR_OFFSET 0x04

#define USART_BRR_OFFSET 0x08

#define USART_CR1_OFFSET 0x0c
#define USART_CR1_UE_BIT 13
#define USART_CR1_M_BIT 12
#define USART_CR1_PCE_BIT 10
#define USART_CR1_PS_BIT 9
#define USART_CR1_TXEIE_BIT 7
#define USART_CR1_TCIE_BIT 6
#define USART_CR1_RXNEIE_BIT 5
#define USART_CR1_TE_BIT 3
#define USART_CR1_RE_BIT 2

#define USART_CR2_OFFSET 0x10
#define USART_CR2_STOP_START 12
#define USART_CR2_STOP_MASK 0x00003000

#define USART_CR3_OFFSET 0x14
#define USART_CR3_CTSE_BIT 9
#define USART_CR3_RTSE_BIT 8

#define USART_GTPR_OFFSET 0x18

#define USART_RCV_BUF_LEN 256

struct Stm32Uart {
    /* Inherited */
    SysBusDevice busdev;

    /* Properties */
    stm32_periph_t periph;
    void *stm32_rcc_prop;
    void *stm32_gpio_prop;
    void *stm32_afio_prop;

    /* Checks the USART transmit pin's GPIO settings.  If the GPIO is not configured
     * properly, a hardware error is triggered.
     */
    union {
        void *check_tx_pin_prop;
        void (*check_tx_pin_callback)(Stm32Uart *);
    };

    /* Private */
    MemoryRegion iomem;

    Stm32Rcc *stm32_rcc;
    Stm32Gpio **stm32_gpio;
    Stm32Afio *stm32_afio;

    int uart_index;

    uint32_t bits_per_sec;
    int64_t ns_per_char;

    /* Register Values */
    uint32_t
        USART_RDR,
        USART_TDR,
        USART_BRR,
        USART_CR1,
        USART_CR2,
        USART_CR3;

    /* Register Field Values */
    uint32_t
        USART_SR_TXE,
        USART_SR_TC,
        USART_SR_RXNE,
        USART_SR_ORE,
        USART_CR1_UE,
        USART_CR1_TXEIE,
        USART_CR1_TCIE,
        USART_CR1_RXNEIE,
        USART_CR1_TE,
        USART_CR1_RE;

    bool sr_read_since_ore_set;

    /* Indicates whether the USART is currently receiving a byte. */
    bool receiving;

    /* Timers used to simulate a delay corresponding to the baud rate. */
    struct QEMUTimer *rx_timer;
    struct QEMUTimer *tx_timer;

    void *chr_write_obj;
    int (*chr_write)(void *chr_write_obj, const uint8_t *buf, int len);

    /* Stores the USART pin mapping used by the board.  This is used to check
     * the AFIO's USARTx_REMAP register to make sure the software has set
     * the correct mapping.
     */
    uint32_t afio_board_map;

    qemu_irq irq;
    int curr_irq_level;

    /* We buffer the characters we receive from our qemu_chr receive handler in here
     * to increase our overall throughput. This allows us to tell the target that
     * another character is ready immediately after it does a read.
     */
    uint8_t rcv_char_buf[USART_RCV_BUF_LEN];
    uint32_t rcv_char_bytes;    /* number of bytes avaialable in rcv_char_buf */
};



/* STM32F1XX Specific stuff */

/* Checks the USART transmit pin's GPIO settings.  If the GPIO is not configured
 * properly, a hardware error is triggered.
 */
void stm32_afio_uart_check_tx_pin_callback(Stm32Uart *s)
{
    int tx_periph, tx_pin;
    int config;

    switch(s->periph) {
        case STM32F1XX_UART1:
            switch(stm32_afio_get_periph_map(s->stm32_afio, s->periph)) {
                case STM32_USART1_NO_REMAP:
                    tx_periph = STM32F1XX_GPIOA;
                    tx_pin = 9;
                    break;
                case STM32_USART1_REMAP:
                    tx_periph = STM32F1XX_GPIOB;
                    tx_pin = 6;
                    break;
                default:
                    assert(false);
                    break;
            }
            break;
        case STM32F1XX_UART2:
            switch(stm32_afio_get_periph_map(s->stm32_afio, s->periph)) {
                case STM32_USART2_NO_REMAP:
                    tx_periph = STM32F1XX_GPIOA;
                    tx_pin = 2;
                    break;
                case STM32_USART2_REMAP:
                    tx_periph = STM32F1XX_GPIOD;
                    tx_pin = 5;
                    break;
                default:
                    assert(false);
                    break;
            }
            break;
        case STM32F1XX_UART3:
            switch(stm32_afio_get_periph_map(s->stm32_afio, s->periph)) {
                case STM32_USART3_NO_REMAP:
                    tx_periph = STM32F1XX_GPIOB;
                    tx_pin = 10;
                    break;
                case STM32_USART3_PARTIAL_REMAP:
                    tx_periph = STM32F1XX_GPIOC;
                    tx_pin = 10;
                    break;
                case STM32_USART3_FULL_REMAP:
                    tx_periph = STM32F1XX_GPIOD;
                    tx_pin = 8;
                    break;
                default:
                    assert(false);
                    break;
            }
            break;
        default:
            assert(false);
            break;
    }

    Stm32Gpio *gpio_dev = s->stm32_gpio[tx_periph - STM32F1XX_GPIOA];

    if(stm32_gpio_get_mode_bits(gpio_dev, tx_pin) == STM32_GPIO_MODE_IN) {
        hw_error("UART TX pin needs to be configured as output");
    }

    config = stm32_gpio_get_config_bits(gpio_dev, tx_pin);
    if((config != STM32_GPIO_OUT_ALT_PUSHPULL) &&
       (config != STM32_GPIO_OUT_ALT_OPEN)) {
        hw_error("UART TX pin needs to be configured as "
                 "alternate function output");
    }
}


/* HELPER FUNCTIONS */

/* Update the baud rate based on the USART's peripheral clock frequency. */
static void stm32_uart_baud_update(Stm32Uart *s)
{
    uint32_t clk_freq = stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph);
    uint64_t ns_per_bit;

    if((s->USART_BRR == 0) || (clk_freq == 0)) {
        s->bits_per_sec = 0;
    } else {
        s->bits_per_sec = clk_freq / s->USART_BRR;
        ns_per_bit = 1000000000LL / s->bits_per_sec;

        /* We assume 10 bits per character.  This may not be exactly
         * accurate depending on settings, but it should be good enough. */
        s->ns_per_char = ns_per_bit * 10;
    }

#ifdef DEBUG_STM32_UART
    const char *periph_name = s->busdev.parent_obj.id;
    DPRINTF("%s clock is set to %lu Hz.\n",
                periph_name,
                (unsigned long)clk_freq);
    DPRINTF("%s BRR set to %lu.\n",
                periph_name,
                (unsigned long)s->USART_BRR);
    DPRINTF("%s Baud is set to %lu bits per sec.\n",
                periph_name,
                (unsigned long)s->bits_per_sec);
#endif
}

/* Handle a change in the peripheral clock. */
static void stm32_uart_clk_irq_handler(void *opaque, int n, int level)
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    assert(n == 0);

    /* Only update the BAUD rate if the IRQ is being set. */
    if(level) {
        stm32_uart_baud_update(s);
    }
}

/* Routine which updates the USART's IRQ.  This should be called whenever
 * an interrupt-related flag is updated.
 */
static void stm32_uart_update_irq(Stm32Uart *s) {
    /* Note that we are not checking the ORE flag, but we should be. */
    int new_irq_level =
       (s->USART_CR1_TCIE & s->USART_SR_TC) |
       (s->USART_CR1_TXEIE & s->USART_SR_TXE) |
       (s->USART_CR1_RXNEIE &
               (s->USART_SR_ORE | s->USART_SR_RXNE));

    /* Only trigger an interrupt if the IRQ level changes.  We probably could
     * set the level regardless, but we will just check for good measure.
     */
    if(new_irq_level ^ s->curr_irq_level) {
        qemu_set_irq(s->irq, new_irq_level);
        s->curr_irq_level = new_irq_level;
    }
}


static void stm32_uart_start_tx(Stm32Uart *s, uint32_t value);

/* Routine to be called when a transmit is complete. */
static void stm32_uart_tx_complete(Stm32Uart *s)
{
    if(s->USART_SR_TXE == 1) {
        /* If the buffer is empty, there is nothing waiting to be transmitted.
         * Mark the transmit complete. */
        s->USART_SR_TC = 1;
        stm32_uart_update_irq(s);
    } else {
        /* Otherwise, mark the transmit buffer as empty and
         * start transmitting the value stored there.
         */
        s->USART_SR_TXE = 1;
        stm32_uart_update_irq(s);
        stm32_uart_start_tx(s, s->USART_TDR);
    }
}

/* Start transmitting a character. */
static void stm32_uart_start_tx(Stm32Uart *s, uint32_t value)
{
    uint8_t ch = value; //This will truncate the ninth bit

    /* Reset the Transmission Complete flag to indicate a transmit is in
     * progress.
     */
    s->USART_SR_TC = 0;

    /* Write the character out. */
    if (s->chr_write_obj) {
        s->chr_write(s->chr_write_obj, &ch, 1);
    }
#ifdef STM32_UART_NO_BAUD_DELAY
    /* If BAUD delays are not being simulated, then immediately mark the
     * transmission as complete.
     */
    stm32_uart_tx_complete(s);
#else
    /* Otherwise, start the transmit delay timer. */
    timer_mod(s->tx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
}


/* Put byte into the receive data register, if we have one and the target is 
 * ready for it. */
static void stm32_uart_fill_receive_data_register(Stm32Uart *s)
{
    bool enabled = (s->USART_CR1_UE && s->USART_CR1_RE);

    /* If we have no more data, or we are emulating baud delay and it's not 
     * time yet for the next byte, return without filling the RDR */
    if (!s->rcv_char_bytes || s->receiving) {
        return;
    }

#ifndef STM32_UART_ENABLE_OVERRUN
    /* If overrun is not enabled, don't overwrite the current byte in the RDR */
    if (enabled && s->USART_SR_RXNE) {
        return;
    }
#endif

    /* Pull the byte out of our buffer */
    uint8_t byte = s->rcv_char_buf[0];
    memmove(&s->rcv_char_buf[0], &s->rcv_char_buf[1], --(s->rcv_char_bytes));

    /* Only handle the received character if the module is enabled, */
    if (enabled) {
        if(s->USART_SR_RXNE) {
            DPRINTF("stm32_uart_receive: overrun error\n");
            s->USART_SR_ORE = 1;
            s->sr_read_since_ore_set = false;
            stm32_uart_update_irq(s);
        }

        /* Receive the character and mark the buffer as not empty. */
        s->USART_RDR = byte;
        s->USART_SR_RXNE = 1;
        stm32_uart_update_irq(s);
    }

#ifndef STM32_UART_NO_BAUD_DELAY
    /* Indicate the module is receiving and start the delay. */
    s->receiving = true;
    timer_mod(s->rx_timer,  qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + s->ns_per_char);
#endif
}



/* TIMER HANDLERS */
/* Once the receive delay is finished, indicate the USART is finished receiving.
 * This will allow it to receive the next character.  The current character was
 * already received before starting the delay.
 */
static void stm32_uart_rx_timer_expire(void *opaque) {
    Stm32Uart *s = (Stm32Uart *)opaque;

    s->receiving = false;

    /* Put next byte into the receive data register, if we have one ready */
    stm32_uart_fill_receive_data_register(s);
}

/* When the transmit delay is complete, mark the transmit as complete
 * (the character was already sent before starting the delay). */
static void stm32_uart_tx_timer_expire(void *opaque) {
    Stm32Uart *s = (Stm32Uart *)opaque;

    stm32_uart_tx_complete(s);
}





/* CHAR DEVICE HANDLERS */

static int stm32_uart_can_receive(void *opaque)
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    /* How much space do we have in our buffer? */
    return (USART_RCV_BUF_LEN - s->rcv_char_bytes);
}

static void stm32_uart_event(void *opaque, int event)
{
    /* Do nothing */
}

static void stm32_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;

    assert(size > 0);

    /* Copy the characters into our buffer first */
    assert (size <= USART_RCV_BUF_LEN - s->rcv_char_bytes);
    memmove(s->rcv_char_buf + s->rcv_char_bytes, buf, size);
    s->rcv_char_bytes += size;

    /* Put next byte into RDR if the target is ready for it */
    stm32_uart_fill_receive_data_register(s);
}




/* REGISTER IMPLEMENTATION */

static uint32_t stm32_uart_USART_SR_read(Stm32Uart *s)
{
    /* If the Overflow flag is set, reading the SR register is the first step
     * to resetting the flag.
     */
    if(s->USART_SR_ORE) {
        s->sr_read_since_ore_set = true;
    }

    return (s->USART_SR_TXE << USART_SR_TXE_BIT) |
           (s->USART_SR_TC << USART_SR_TC_BIT) |
           (s->USART_SR_RXNE << USART_SR_RXNE_BIT) |
           (s->USART_SR_ORE << USART_SR_ORE_BIT);
}





static void stm32_uart_USART_SR_write(Stm32Uart *s, uint32_t new_value)
{
    uint32_t new_TC, new_RXNE;

    new_TC = extract32(new_value, USART_SR_TC_BIT, 1);
    /* The Transmit Complete flag can be cleared, but not set. */
    if(new_TC) {
        hw_error("Software attempted to set USART TC bit\n");
    }
    s->USART_SR_TC = new_TC;

    new_RXNE = extract32(new_value, USART_SR_RXNE_BIT, 1);
    /* The Read Data Register Not Empty flag can be cleared, but not set. */
    if(new_RXNE) {
        hw_error("Software attempted to set USART RXNE bit\n");
    }
    s->USART_SR_RXNE = new_RXNE;

    stm32_uart_update_irq(s);
}


static void stm32_uart_USART_DR_read(Stm32Uart *s, uint32_t *read_value)
{
    /* If the Overflow flag is set, then it should be cleared if the software
     * performs an SR read followed by a DR read.
     */
    if(s->USART_SR_ORE) {
        if(s->sr_read_since_ore_set) {
            s->USART_SR_ORE = 0;

        }
    }

    if(!s->USART_CR1_UE) {
        hw_error("Attempted to read from USART_DR while UART was disabled.");
    }

    if(!s->USART_CR1_RE) {
        hw_error("Attempted to read from USART_DR while UART receiver "
                 "was disabled.");
    }

    if(s->USART_SR_RXNE) {
        /* If the receive buffer is not empty, return the value. and mark the
         * buffer as empty.
         */
        (*read_value) = s->USART_RDR;
        s->USART_SR_RXNE = 0;

        /* Put next character into the RDR if we have one */
        stm32_uart_fill_receive_data_register(s);
    } else {
        printf("STM32_UART WARNING: Read value from USART_DR while it was empty.\n");
    }

    stm32_uart_update_irq(s);
}


static void stm32_uart_USART_DR_write(Stm32Uart *s, uint32_t new_value)
{
    uint32_t write_value = new_value & 0x000001ff;

    if(!s->USART_CR1_UE) {
        hw_error("Attempted to write to USART_DR while UART was disabled.");
    }

    if(!s->USART_CR1_TE) {
        hw_error("Attempted to write to USART_DR while UART transmitter "
                 "was disabled.");
    }

    if (s->check_tx_pin_callback) {
        s->check_tx_pin_callback(s);
    }

    if(s->USART_SR_TC) {
        /* If the Transmission Complete bit is set, it means the USART is not
         * currently transmitting.  This means, a transmission can immediately
         * start.
         */
        stm32_uart_start_tx(s, write_value);
    } else {
        /* Otherwise check to see if the buffer is empty.
         * If it is, then store the new character there and mark it as not empty.
         * If it is not empty, trigger a hardware error.  Software should check
         * to make sure it is empty before writing to the Data Register.
         */
        if(s->USART_SR_TXE) {
            s->USART_TDR = write_value;
            s->USART_SR_TXE = 0;
        } else {
            hw_error("Wrote new value to USART_DR while it was non-empty.");
        }
    }

    stm32_uart_update_irq(s);
}

/* Update the Baud Rate Register. */
static void stm32_uart_USART_BRR_write(Stm32Uart *s, uint32_t new_value,
                                        bool init)
{
    s->USART_BRR = new_value & 0x0000ffff;

    stm32_uart_baud_update(s);
}

static void stm32_uart_USART_CR1_write(Stm32Uart *s, uint32_t new_value,
                                        bool init)
{
    s->USART_CR1_UE = extract32(new_value, USART_CR1_UE_BIT, 1);
#if 0 /* XXX Does not work with f2xx yet */
    if(s->USART_CR1_UE) {
        /* Check to make sure the correct mapping is selected when enabling the
         * USART.
         */
        if(s->afio_board_map != stm32_afio_get_periph_map(s->stm32_afio, s->periph)) {
            hw_error("Bad AFIO mapping for %s", s->busdev.parent_obj.id);
        }
    }
#endif

    s->USART_CR1_TXEIE = extract32(new_value, USART_CR1_TXEIE_BIT, 1);
    s->USART_CR1_TCIE = extract32(new_value, USART_CR1_TCIE_BIT, 1);
    s->USART_CR1_RXNEIE = extract32(new_value, USART_CR1_RXNEIE_BIT, 1);

    s->USART_CR1_TE = extract32(new_value, USART_CR1_TE_BIT, 1);
    s->USART_CR1_RE = extract32(new_value, USART_CR1_RE_BIT, 1);

    s->USART_CR1 = new_value & 0x00003fff;

    stm32_uart_update_irq(s);
}

static void stm32_uart_USART_CR2_write(Stm32Uart *s, uint32_t new_value,
                                        bool init)
{
    s->USART_CR2 = new_value & 0x00007f7f;
}

static void stm32_uart_USART_CR3_write(Stm32Uart *s, uint32_t new_value,
                                        bool init)
{
    s->USART_CR3 = new_value & 0x000007ff;
}

static void stm32_uart_reset(DeviceState *dev)
{
    Stm32Uart *s = STM32_UART(dev);

    /* Initialize the status registers.  These are mostly
     * read-only, so we do not call the "write" routine
     * like normal.
     */
    s->USART_SR_TXE = 1;
    s->USART_SR_TC = 1;
    s->USART_SR_RXNE = 0;
    s->USART_SR_ORE = 0;

    // Do not initialize USART_DR - it is documented as undefined at reset
    // and does not behave like normal registers.
    stm32_uart_USART_BRR_write(s, 0x00000000, true);
    stm32_uart_USART_CR1_write(s, 0x00000000, true);
    stm32_uart_USART_CR2_write(s, 0x00000000, true);
    stm32_uart_USART_CR3_write(s, 0x00000000, true);

    stm32_uart_update_irq(s);
}

static uint64_t stm32_uart_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    uint32_t value=0;
    int start = (offset & 3) * 8;
    int length = size * 8;

    switch (offset & 0xfffffffc) {
        case USART_SR_OFFSET:
            return extract64(stm32_uart_USART_SR_read(s), start, length);
        case USART_DR_OFFSET:
            stm32_uart_USART_DR_read(s, &value);
            return extract64(value, start, length);
        case USART_BRR_OFFSET:
            return extract64(s->USART_BRR, start, length);
        case USART_CR1_OFFSET:
            return extract64(s->USART_CR1, start, length);
        case USART_CR2_OFFSET:
            return extract64(s->USART_CR2, start, length);
        case USART_CR3_OFFSET:
            return extract64(s->USART_CR3, start, length);
        case USART_GTPR_OFFSET:
            STM32_NOT_IMPL_REG(offset, size);
            return 0;
        default:
            STM32_BAD_REG(offset, size);
            return 0;
    }
}

static void stm32_uart_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    Stm32Uart *s = (Stm32Uart *)opaque;
    int start = (offset & 3) * 8;
    int length = size * 8;

    stm32_rcc_check_periph_clk((Stm32Rcc *)s->stm32_rcc, s->periph);

    switch (offset & 0xfffffffc) {
        case USART_SR_OFFSET:
            stm32_uart_USART_SR_write(s,
                  deposit64(stm32_uart_USART_SR_read(s), start, length, value));
            break;
        case USART_DR_OFFSET:
            stm32_uart_USART_DR_write(s,
                  deposit64(0, start, length, value));
            break;
        case USART_BRR_OFFSET:
            stm32_uart_USART_BRR_write(s,
                  deposit64(s->USART_BRR, start, length, value), false);
            break;
        case USART_CR1_OFFSET:
            stm32_uart_USART_CR1_write(s,
                  deposit64(s->USART_CR1, start, length, value), false);
            break;
        case USART_CR2_OFFSET:
            stm32_uart_USART_CR2_write(s,
                  deposit64(s->USART_CR2, start, length, value), false);
            break;
        case USART_CR3_OFFSET:
            stm32_uart_USART_CR3_write(s,
                  deposit64(s->USART_CR3, start, length, value), false);
            break;
        case USART_GTPR_OFFSET:
            STM32_NOT_IMPL_REG(offset, 2);
            break;
        default:
            STM32_BAD_REG(offset, 2);
            break;
    }
}

static const MemoryRegionOps stm32_uart_ops = {
    .read = stm32_uart_read,
    .write = stm32_uart_write,
    .valid.min_access_size = 2,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN
};





/* PUBLIC FUNCTIONS */

void stm32_uart_set_write_handler(Stm32Uart *s, void *obj,
        int (*chr_write_handler)(void *chr_write_obj, const uint8_t *buf, int len))
{
    s->chr_write_obj = obj;
    s->chr_write = chr_write_handler;
}


void stm32_uart_get_rcv_handlers(Stm32Uart *s, IOCanReadHandler **can_read,
                                 IOReadHandler **read, IOEventHandler **event)
{
    *can_read = stm32_uart_can_receive;
    *read = stm32_uart_receive;
    *event = stm32_uart_event;
}


// Stub used to typecast the generic write handler prototype to a
// qemu_chr write handler.
static int stm32_uart_chr_fe_write_stub(void *s, const uint8_t *buf, int len)
{
    return qemu_chr_fe_write((CharDriverState *)s, buf, len);
}


// Helper method that connects this UART device's receive handlers to a qemu_chr instance.
void stm32_uart_connect(Stm32Uart *s, CharDriverState *chr, uint32_t afio_board_map)
{
    s->chr_write_obj = chr;
    if (chr) {
        stm32_uart_set_write_handler(s, chr, stm32_uart_chr_fe_write_stub);
        IOCanReadHandler *can_read_cb;
        IOReadHandler *read_cb;
        IOEventHandler *event_cb;
        stm32_uart_get_rcv_handlers(s, &can_read_cb, &read_cb, &event_cb);
        qemu_chr_add_handlers(chr, can_read_cb, read_cb, event_cb, s);
    }

    s->afio_board_map = afio_board_map;
}




/* DEVICE INITIALIZATION */

static int stm32_uart_init(SysBusDevice *dev)
{
    qemu_irq *clk_irq;
    Stm32Uart *s = STM32_UART(dev);

    s->stm32_rcc = (Stm32Rcc *)s->stm32_rcc_prop;
    s->stm32_gpio = (Stm32Gpio **)s->stm32_gpio_prop;
    s->stm32_afio = (Stm32Afio *)s->stm32_afio_prop;

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32_uart_ops, s,
                          "uart", 0x03ff);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    s->rx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_uart_rx_timer_expire, s);
    s->tx_timer =
        timer_new_ns(QEMU_CLOCK_VIRTUAL,
                  (QEMUTimerCB *)stm32_uart_tx_timer_expire, s);

    /* Register handlers to handle updates to the USART's peripheral clock. */
    clk_irq =
          qemu_allocate_irqs(stm32_uart_clk_irq_handler, (void *)s, 1);
    stm32_rcc_set_periph_clk_irq(s->stm32_rcc, s->periph, clk_irq[0]);

    s->rcv_char_bytes = 0;

    stm32_uart_reset((DeviceState *)s);

    return 0;
}

static Property stm32_uart_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Uart, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_PTR("stm32_rcc", Stm32Uart, stm32_rcc_prop),
    DEFINE_PROP_PTR("stm32_gpio", Stm32Uart, stm32_gpio_prop),
    DEFINE_PROP_PTR("stm32_afio", Stm32Uart, stm32_afio_prop),
    DEFINE_PROP_PTR("stm32_check_tx_pin_callback", Stm32Uart, check_tx_pin_prop),
    DEFINE_PROP_END_OF_LIST()
};

static void stm32_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_uart_init;
    dc->reset = stm32_uart_reset;
    dc->props = stm32_uart_properties;
}

static TypeInfo stm32_uart_info = {
    .name  = "stm32-uart",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32Uart),
    .class_init = stm32_uart_class_init
};

static void stm32_uart_register_types(void)
{
    type_register_static(&stm32_uart_info);
}

type_init(stm32_uart_register_types)
