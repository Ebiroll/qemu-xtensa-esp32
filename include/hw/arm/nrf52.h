/*
 * Nordic Semiconductor nRF51 Series SOC Common Defines
 *
 * This file hosts generic defines used in various nRF51 peripheral devices.
 *
 * Reference Manual: http://infocenter.nordicsemi.com/pdf/nRF51_RM_v3.0.pdf
 * Product Spec: http://infocenter.nordicsemi.com/pdf/nRF51822_PS_v3.1.pdf
 *
 * Copyright 2018 Steffen Görtz <contrib@steffen-goertz.de>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#ifndef NRF52_H
#define NRF52_H

// https://electronut.in/nrf52-baremetal/

#define NRF52_FLASH_BASE      0x00000000
#define NRF52_FICR_BASE       0x10000000
#define NRF52_FICR_SIZE       0x00000100
#define NRF52_UICR_BASE       0x10001000
#define NRF52_SRAM_BASE       0x20000000

#define NRF52_IOMEM_BASE      0x40000000
#define NRF52_IOMEM_SIZE      0x20000000

#define NRF52_UART_BASE       0x40002000
#define NRF52_TWI_BASE        0x40003000
#define NRF52_TWI_SIZE        0x00002000

#define NRF52_TIMER_BASE      0x40008000
#define NRF52_TIMER_SIZE      0x00001000
#define NRF52_RNG_BASE        0x4000D000
#define NRF52_NVMC_BASE       0x4001E000
#define NRF52_GPIO_BASE       0x50000000

#define NRF52_PRIVATE_BASE    0xF0000000
#define NRF52_PRIVATE_SIZE    0x10000000

#define NRF52_PAGE_SIZE       1024

/* Trigger */
#define NRF52_TRIGGER_TASK 0x01

/* Events */
#define NRF52_EVENT_CLEAR  0x00

#endif
