/*
 * Nordic Semiconductor nRF52  SoC
 *
 * Copyright 2018 Joel Stanley <joel@jms.id.au>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef NRF52_SOC_H
#define NRF52_SOC_H

#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
#include "hw/char/nrf51_uart.h"
#include "hw/misc/nrf51_rng.h"
#include "hw/gpio/nrf51_gpio.h"
#include "hw/nvram/nrf51_nvm.h"
#include "hw/timer/nrf51_timer.h"

#define TYPE_NRF52_SOC "nrf52-soc"
#define NRF52_SOC(obj) \
    OBJECT_CHECK(NRF52State, (obj), TYPE_NRF52_SOC)

#define NRF52_NUM_TIMERS 3

typedef struct NRF52State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    ARMv7MState cpu;

    NRF51UARTState uart;
    NRF51RNGState rng;
    NRF51NVMState nvm;
    NRF51GPIOState gpio;
    NRF51TimerState timer[NRF52_NUM_TIMERS];

    MemoryRegion iomem;
    MemoryRegion sram;
    MemoryRegion flash;
    MemoryRegion clock;
    MemoryRegion twi;

    uint32_t sram_size;
    uint32_t flash_size;

    MemoryRegion *board_memory;

    MemoryRegion container;

} NRF52State;

#endif

