#ifndef __NRF51_H__
#define __NRF51_H__
#include "exec/address-spaces.h"

#define TIMER_LOG_ENABLED 0


#define SRAM_32K (32 * 1024)
#define SIZERAMBLOCKS (8192)
#define NUMRAMBLOCK (SRAM_32K / SIZERAMBLOCKS)
#define NRF51_SRAM_BASE (0x20000000)
#define DEVICEID_HI (0x25411fc2)
#define DEVICEID_LO (0x8d136c9c)
#define DEVICEADDR_LO (0x87a4a811)
#define DEVICEADDR_HI (0xeb96)
#define DEVICEADDRTYPE_RANDOM 1
#define DEVICEADDRTYPE_PUBLIC 0

/*************************************\
 * Base Addresses
\*************************************/
#define GPIO_BASE (0x50000000)
#define GPIO_REG_SPACE (0x780)


#define GPTE_BASE (0x40006000)
#define GPTE_REG_SPACE (0x520)

//Power, Clock and MPU share the same peripheral
#define POWER_CLOCK_BASE (0x40000000)
#define POWER_CLOCK_REG_SPACE (0x610) //Space for all 3
#define IRQ_POWER_CLOCK 8

#define RTC0_BASE (0x4000B000)
#define RTC1_BASE (0x40011000)
#define RTC_REG_SPACE (0x550)
#define RTC_TOTAL 2

#define UART_BASE (0x40002000)
#define UART_REG_SPACE (0x570)

#define ECB_BASE (0x4000E000)
#define ECB_REG_SPACE (0x508)

#define ADC_BASE (0x40007000)
#define ADC_REG_SPACE (0x50C)

#define RADIO_BASE (0x40001000)
#define RADIO_REG_SPACE (0x1000)

#define TIMER0_BASE (0x40008000)
#define TIMER1_BASE (0x40009000)
#define TIMER2_BASE (0x4000A000)
#define TIMER_REG_SPACE (0x550)
#define TIMER_TOTAL 3

#define RNG_BASE (0x4000D000)
#define RNG_REG_SPACE (0x50C)

#define NVM_BASE (0x4001E000)
#define NVM_REG_SPACE (0x518)

#define FICR_BASE (0x10000000)
#define FICR_REG_SPACE (0x100)

#define UICR_BASE (0x10001000)
#define UICR_REG_SPACE (0x100)

#define WDT_BASE (0x40010000)
#define WDT_REG_SPACE (0x620)

#define PPI_BASE (0x4001F000)
#define PPI_REG_SPACE (0x810)

//TODO: [WARNING] Same space is shared by other peripherals.
#define CCM_BASE (0x4000F000)
#define CCM_REG_SPACE (0x518)

/*************************************\
 * Offsets
\*************************************/
enum
{
    O_GPIO_OUT = 0x504,
    O_GPIO_OUTSET = 0x508,
    O_GPIO_OUTCLR = 0x50C,
    O_GPIO_IN = 0x510,
    O_GPIO_DIR = 0x514,
    O_GPIO_DIRSET = 0x518,
    O_GPIO_DIRCLR = 0x51C,
    O_GPIO_PIN_CNF0 = 0x700,
    O_GPIO_PIN_CNF31 = 0x77C
};

enum
{
    //Tasks
    O_GPTE_OUT0 = 0x0,
    O_GPTE_OUT1 = 0x4,
    O_GPTE_OUT2 = 0x8,
    O_GPTE_OUT3 = 0xC,

    //Events
    O_GPTE_IN0  = 0x100,
    O_GPTE_IN1  = 0x104,
    O_GPTE_IN2  = 0x108,
    O_GPTE_IN3  = 0x10C,
    O_GPTE_PORT = 0x17C, //FIXME: PPI

    //Registers
    O_GPTE_INTEN    = 0x300,
    O_GPTE_INTENSET = 0x304,
    O_GPTE_INTENCLR = 0x308,
    O_GPTE_CONFIG0  = 0x510,
    O_GPTE_CONFIG1  = 0x514,
    O_GPTE_CONFIG2  = 0x518,
    O_GPTE_CONFIG3  = 0x51C,
};

enum
{
    //Tasks
    O_CLOCK_HFCLKSTART = 0x0,
    O_CLOCK_HFCLKSTOP  = 0x4,
    O_CLOCK_LFCLKSTART = 0x8,
    O_CLOCK_LFCLKSTOP  = 0xC,
    O_CLOCK_CAL        = 0x10,
    O_CLOCK_CTSTART    = 0x14,
    O_CLOCK_CTSTOP     = 0x18,

    //Events
    O_CLOCK_HFCLKSTARTED = 0x100,
    O_CLOCK_LFCLKSTARTED = 0x104,
    O_CLOCK_DONE         = 0x10C,
    O_CLOCK_CTTO         = 0x110,

    //Registers
    O_CLOCK_INTENSET     = 0x304,
    O_CLOCK_INTENCLR     = 0x308,
    O_CLOCK_HFCLKRUN     = 0x408,
    O_CLOCK_HFCLKSTAT    = 0x40C,
    O_CLOCK_LFCLKRUN     = 0x414,
    O_CLOCK_LFCLKSTAT    = 0x418,
    O_CLOCK_LFCLKSRCCOPY = 0x41C,
    O_CLOCK_LFCLKSRC     = 0x518,
    O_CLOCK_CTIV         = 0x538,
    O_CLOCK_XTALFREQ     = 0x550
};

enum
{
    //Tasks
    O_RTC_START = 0x0,
    O_RTC_STOP = 0x4,
    O_RTC_CLEAR = 0x8,
    O_RTC_TRIGOVRFLW = 0xC,

    //Events
    O_RTC_TICK = 0x100,
    O_RTC_OVRFLW = 0x104,
    O_RTC_COMPARE0 = 0x140,
    O_RTC_COMPARE1 = 0x144,
    O_RTC_COMPARE2 = 0x148,
    O_RTC_COMPARE3 = 0x14C,

    //Registers
    O_RTC_INTEN     = 0x300,
    O_RTC_INTENSET  = 0x304,
    O_RTC_INTENCLR  = 0x308,
    O_RTC_EVTEN     = 0x340,
    O_RTC_EVTENSET  = 0x344,
    O_RTC_EVTENCLR  = 0x348,
    O_RTC_COUNTER   = 0x504,
    O_RTC_PRESCALER = 0x508,
    O_RTC_CC0       = 0x540,
    O_RTC_CC1       = 0x544,
    O_RTC_CC2       = 0x548,
    O_RTC_CC3       = 0x54c,
};

enum
{
    //Tasks
    O_UART_STARTRX  = 0x000,
    O_UART_STOPRX   = 0x004,
    O_UART_STARTTX  = 0x008,
    O_UART_STOPTX   = 0x00C,
    O_UART_SUSPEND  = 0x01C,

    //Events
    O_UART_CTS      = 0x100,
    O_UART_NCTS     = 0x104,
    O_UART_RXDRDY   = 0x108,
    O_UART_TXDRDY   = 0x11C,
    O_UART_ERROR    = 0x124,
    O_UART_RXTO     = 0x144,

    //Registers
    O_UART_INTEN    = 0x300,
    O_UART_INTENSET = 0x304,
    O_UART_INTENCLR = 0x308,
    O_UART_ERRORSRC = 0x480,
    O_UART_ENABLE   = 0x500,
    O_UART_PSELRTS  = 0x508,
    O_UART_PSELTXD  = 0x50C,
    O_UART_PSELCTS  = 0x510,
    O_UART_PSELRXD  = 0x514,
    O_UART_RXD      = 0x518,
    O_UART_TXD      = 0x51C,
    O_UART_BAUDRATE = 0x524,
    O_UART_CONFIG   = 0x56C
};

enum
{
    //Tasks
    O_ADC_START = 0x0,
    O_ADC_STOP = 0x04,

    //Events
    O_ADC_END = 0x100,

    //Registers
    O_ADC_INTEN = 0x300,
    O_ADC_INTENSET = 0x304,
    O_ADC_INTENCLR = 0x308,
    O_ADC_BUSY = 0x400,
    O_ADC_ENABLE = 0x500,
    O_ADC_CONFIG = 0x504,
    O_ADC_RESULT = 0x508,
};

enum
{
    //Tasks
    O_RNG_START = 0x0,
    O_RNG_STOP = 0x4,

    //Event
    O_RNG_VALRDY = 0x100,

    //Registers
    O_RNG_SHORTS = 0x200,
    O_RNG_INTEN = 0x300,
    O_RNG_INTENSET = 0x304,
    O_RNG_INTENCLR = 0x308,
    O_RNG_CONFIG = 0x504,
    O_RNG_VALUE = 0x508
};

enum
{
    //Tasks
    O_ECB_STARTECB = 0x0,
    O_ECB_STOPECB = 0x4,

    //Events
    O_ECB_ENDECB = 0x100,
    O_ECB_ERRORECB = 0x104,

    //Registers
    O_ECB_INTENSET = 0x304,
    O_ECB_INTENCLR = 0x308,
    O_ECB_ECBDATAPTR = 0x504
};

enum
{
    O_NVM_READY = 0x400,
    O_NVM_CONFIG = 0x504,
    O_NVM_ERASEPAGE = 0x508,
    O_NVM_ERASEPCR1 = O_NVM_ERASEPAGE, //Unusued, same in NRF51 SDK
    O_NVM_ERASEALL = 0x50C,
    O_NVM_ERASEPCR0 = 0x510,
    O_NVM_ERASEUICR = 0x514
};

enum
{
    O_FICR_CODEPAGESIZE = 0x10,
    O_FICR_CODESIZE = 0x14,
    O_FICR_CLENR0 = 0x28,
    O_FICR_PPFC = 0x2C,
    O_FICR_NUMRAMBLOCK = 0x34,
    O_FICR_SIZERAMBLOCK0 = 0x38,
    O_FICR_SIZERAMBLOCK1 = 0x3C, //Deprecated
    O_FICR_SIZERAMBLOCK2 = 0x40, //Deprecated
    O_FICR_SIZERAMBLOCK3 = 0x44, //Deprecated
    O_FICR_SIZERAMBLOCKS = O_FICR_SIZERAMBLOCK0,
    O_FICR_CONFIGID = 0x5C,
    O_FICR_DEVICEID0 = 0x60,
    O_FICR_DEVICEID1 = 0x64,
    O_FICR_ER0 = 0x80,
    O_FICR_ER1 = 0x84,
    O_FICR_ER2 = 0x88,
    O_FICR_ER3 = 0x8C,
    O_FICR_IR0 = 0x90,
    O_FICR_IR1 = 0x94,
    O_FICR_IR2 = 0x98,
    O_FICR_IR3 = 0x9C,
    O_FICR_DEVICEADDRTYPE = 0xA0,
    O_FICR_DEVICEADDR0 = 0xA4,
    O_FICR_DEVICEADDR1 = 0xA8,
    O_FICR_OVERRIDEEN = 0xAC,
    O_FICR_NRF_1MBIT0 = 0xB0,
    O_FICR_NRF_1MBIT1 = 0xB4,
    O_FICR_NRF_1MBIT2 = 0xB8,
    O_FICR_NRF_1MBIT3 = 0xBC,
    O_FICR_NRF_1MBIT4 = 0xC0,
    O_FICR_BLE_1MBIT0 = 0xEC,
    O_FICR_BLE_1MBIT1 = 0xF0,
    O_FICR_BLE_1MBIT2 = 0xF4,
    O_FICR_BLE_1MBIT3 = 0xF8,
    O_FICR_BLE_1MBIT4 = 0xFC
};

enum
{
    O_UICR_CLENR0         = 0x00,
    O_UICR_RBPCONF        = 0x04,
    O_UICR_XTALFREQ       = 0x08,
    O_UICR_FWID           = 0x10,
    O_UICR_BOOTLOADERADDR = 0x14,
    O_UICR_NRFFW1         = 0x18,
    O_UICR_NRFFW14        = 0x4C,
    O_UICR_NRFHW1         = 0x50,
    O_UICR_NRFHW11        = 0x7C,
    O_UICR_CUSTOMER0      = 0x80,
    O_UICR_CUSTOMER31     = 0xFC,
};

enum
{
    //Tasks
    O_TIMER_START   = 0x000,
    O_TIMER_STOP    = 0x004,
    O_TIMER_COUNT   = 0x008,
    O_TIMER_CLEAR   = 0x00C,
    O_TIMER_SHUTDOWN = 0x010,
    O_TIMER_CAPTURE0 = 0x040,
    O_TIMER_CAPTURE1 = 0x044,
    O_TIMER_CAPTURE2 = 0x048,
    O_TIMER_CAPTURE3 = 0x04C,

    //Events
    O_TIMER_COMPARE0 = 0x140,
    O_TIMER_COMPARE1 = 0x144,
    O_TIMER_COMPARE2 = 0x148,
    O_TIMER_COMPARE3 = 0x14C,

    //Registers
    O_TIMER_SHORTS    = 0x200,
    O_TIMER_INTENSET  = 0x304,
    O_TIMER_INTENCLR  = 0x308,
    O_TIMER_MODE      = 0x504,
    O_TIMER_BITMODE   = 0x508,
    O_TIMER_PRESCALER = 0x510,
    O_TIMER_CC0       = 0x540,
    O_TIMER_CC1       = 0x544,
    O_TIMER_CC2       = 0x548,
    O_TIMER_CC3       = 0x54C,

};

enum
{
    //Tasks
    O_WDT_START     = 0x000,
    //Events
    O_WDT_TIMEOUT   = 0x100,
    //Registers
    O_WDT_INTENSET  = 0x304,
    O_WDT_INTENCLR  = 0x308,
    O_WDT_RUNSTATUS = 0x400,
    O_WDT_REQSTATUS = 0x404,
    O_WDT_CRV       = 0x504,
    O_WDT_RREN      = 0x508,
    O_WDT_CONFIG    = 0x50C,
    O_WDT_RR0       = 0x600,
    O_WDT_RR1       = 0x604,
    O_WDT_RR2       = 0x608,
    O_WDT_RR3       = 0x60C,
    O_WDT_RR4       = 0x610,
    O_WDT_RR5       = 0x614,
    O_WDT_RR6       = 0x618,
    O_WDT_RR7       = 0x61C
};

enum
{
    //Tasks
    O_PPI_CHG_0_EN  = 0x000,
    O_PPI_CHG_0_DIS = 0x004,
    O_PPI_CHG_1_EN  = 0x008,
    O_PPI_CHG_1_DIS = 0x00C,
    O_PPI_CHG_2_EN  = 0x010,
    O_PPI_CHG_2_DIS = 0x014,
    O_PPI_CHG_3_EN  = 0x018,
    O_PPI_CHG_3_DIS = 0x01C,

    //Registers
    O_PPI_CHEN      = 0x500,
    O_PPI_CHENSET   = 0x504,
    O_PPI_CHENCLR   = 0x508,
    O_PPI_CH_0_EEP  = 0x510, //EEP register of the first channel.
    O_PPI_CH_15_TEP = 0x58C, //TEP register of the last channel.
    O_PPI_CHG_0     = 0x800,
    O_PPI_CHG_1     = 0x804,
    O_PPI_CHG_2     = 0x808,
    O_PPI_CHG_3     = 0x80C,
};

enum
{
    //Task
    O_CCM_KSGEN = 0x000,
    O_CCM_CRYPT = 0x004,
    O_CCM_STOP  = 0x008,

    //Events
    O_CCM_ENDKSGEN = 0x100,
    O_CCM_ENDCRYPT = 0x104,
    O_CCM_ERROR    = 0x108,

    //Registers
    O_CCM_SHORTS     = 0x200,
    O_CCM_INTENSET   = 0x304,
    O_CCM_INTENCLR   = 0x308,
    O_CCM_MICSTATUS  = 0x400,
    O_CCM_ENABLE     = 0x500,
    O_CCM_MODE       = 0x504,
    O_CCM_CNFPTR     = 0x508,
    O_CCM_INPTR      = 0x50C,
    O_CCM_OUTPTR     = 0x510,
    O_CCM_SCRATCHPTR = 0x514
};

/*************************************\
 * IRQs
\*************************************/
#define IRQ_ADC     7
#define IRQ_RTC0    11
#define IRQ_RTC1    17
#define IRQ_GPTE    6
#define IRQ_UART    2
#define IRQ_ECB     14
#define IRQ_CCM_AAR 15
#define IRQ_RADIO   1
#define IRQ_TIMER0  8
#define IRQ_TIMER1  9
#define IRQ_TIMER2  10
#define IRQ_RNG     13

/*************************************\
 * Register Masks
\*************************************/
#define MASK_RTC_INTEN_TICK 0x1
#define MASK_RTC_INTEN_COMPARE0 (1<<16)
#define MASK_RTC_INTEN_COMPARE1 (1<<17)
#define MASK_RTC_INTEN_COMPARE2 (1<<18)
#define MASK_RTC_INTEN_COMPARE3 (1<<19)
#define MASK_RTC_INTEN_OVERFLW  (1<<1) //0x2

//#define MASK_RTC_INTEN_ALLCOMPARE ((1<<16) | (1<<17) | (1<<18) | (1<<19))

#define MASK_RTC_COUNTER ((1<<24) - 1)

#define MASK_GPIO_PINCNF_DIR   (0x1)
#define MASK_GPIO_PINCNF_INPUT (0x2)

#define MASK_CLOCK_LFCLKSRC (0x3)

/*************************************\
 * Register Single Bits (Bit Pos.)
\*************************************/
#define BIT_GPIO_PINCNF_DIR (0)
#define BIT_GPTE_CONFIG_OUTINIT (20)
#define BIT_CLOCK_INT_HFCLKSTARTED (0)
#define BIT_CLOCK_INT_LFCLKSTARTED (1)
#define BIT_CLOCK_INT_DONE (3)
#define BIT_CLOCK_INT_CTTO (4)

/*************************************\
 * Device Configuration Related Enums
\*************************************/
enum
{
    GPTE_CONFIG_DISABLED = 0x0,
    GPTE_CONFIG_EVENT    = 0x1,
    GPTE_CONFIG_TASK     = 0x3
};

enum
{
    GPTE_CONFIG_NONE    = 0x0,
    GPTE_CONFIG_RISING  = 0x1,
    GPTE_CONFIG_FALLING = 0x2,
    GPTE_CONFIG_TOGGLE  = 0x3
};

/*************************************\
 * QEMU Macros
\*************************************/
#define NRF51_GPIO_STATE(obj) \
    OBJECT_CHECK(nrf51_gpio_state, (obj), TYPE_NRF51_GPIO)

#define NRF51_GPTE_STATE(obj) \
    OBJECT_CHECK(nrf51_gpte_state, (obj), TYPE_NRF51_GPTE)

#define NRF51_CLOCK_STATE(obj) \
    OBJECT_CHECK(nrf51_clock_state, (obj), TYPE_NRF51_CLOCK)

#define NRF51_RTC_STATE(obj) \
    OBJECT_CHECK(nrf51_rtc_state, (obj), TYPE_NRF51_RTC)

#define NRF51_ADC_STATE(obj) \
    OBJECT_CHECK(nrf51_adc_state, (obj), TYPE_NRF51_ADC)

#define NRF51_UART_STATE(obj) \
    OBJECT_CHECK(nrf51_uart_state, (obj), TYPE_NRF51_UART)

#define NRF51_ECB_STATE(obj) \
    OBJECT_CHECK(nrf51_ecb_state, (obj), TYPE_NRF51_ECB)

#define NRF51_RADIO_STATE(obj) \
    OBJECT_CHECK(nrf51_radio_state, (obj), TYPE_NRF51_RADIO)

#define NRF51_TIMER_STATE(obj) \
    OBJECT_CHECK(nrf51_timer_state, (obj), TYPE_NRF51_TIMER)

#define NRF51_RNG_STATE(obj) \
    OBJECT_CHECK(nrf51_rng_state, (obj), TYPE_NRF51_RNG)

#define NRF51_NVM_STATE(obj) \
    OBJECT_CHECK(nrf51_nvm_state, (obj), TYPE_NRF51_NVM)

#define NRF51_FICR_STATE(obj) \
    OBJECT_CHECK(nrf51_ficr_state, (obj), TYPE_NRF51_FICR)

#define NRF51_UICR_STATE(obj) \
    OBJECT_CHECK(nrf51_uicr_state, (obj), TYPE_NRF51_UICR)

#define NRF51_WDT_STATE(obj) \
    OBJECT_CHECK(nrf51_wdt_state, (obj), TYPE_NRF51_WDT)

#define NRF51_PPI_STATE(obj) \
    OBJECT_CHECK(nrf51_ppi_state, (obj), TYPE_NRF51_PPI)

#define NRF51_CCM_STATE(obj) \
    OBJECT_CHECK(nrf51_ccm_state, (obj), TYPE_NRF51_CCM)

/*************************************\
 * General Macros
\*************************************/
#define NUM_GPIO_PINS (32)
/*
 * Initialize device state without overwriting
 * first SysBusDevice field.
 */
#define INIT_DEV_STATE(s) memset(((void*)s) + sizeof(SysBusDevice), 0x00, sizeof(*s) - sizeof(SysBusDevice))

#define GET_BIT(val,bit) ( ((val) >> bit) & 0x1 )

// Check if given data pointer and its size is out of RAM bounds
#define NRF51_OUT_OF_RAM(ptr,size) (ptr < NRF51_SRAM_BASE || ptr + size > NRF51_SRAM_BASE + SRAM_32K)

/*************************************\
 * UDP Protocol
\*************************************/
#define PROTO_HDR_SZ (sizeof (nrf51_udp_proto_hdr))

enum
{
    //Note: order is vital.
    PROTO_SEND_ID,
    PROTO_RADIO,
    PROTO_GPIO,
    PROTO_TOTAL
};

typedef struct
{
    uint8_t proto_type;
    uint8_t reserved;
    uint8_t lenHi;
    uint8_t lenLow;
} nrf51_udp_proto_hdr;

typedef struct
{
    nrf51_udp_proto_hdr proto_hdr;
    uint8_t pin_state;
} nrf51_udp_gpio_hdr;

typedef struct
{
    nrf51_udp_proto_hdr proto_hdr;
    uint8_t id[2];
} nrf51_udp_send_id_hdr;

//Make sure that headers are aligned properly.
QEMU_BUILD_BUG_ON(sizeof(nrf51_udp_proto_hdr) != 4);
QEMU_BUILD_BUG_ON(sizeof(nrf51_udp_gpio_hdr) != sizeof(nrf51_udp_proto_hdr) + 1);
QEMU_BUILD_BUG_ON(sizeof(nrf51_udp_send_id_hdr) != sizeof(nrf51_udp_proto_hdr) + 2);

//Calculate the header size for a specific struct.
//It doesn't apply to messages with variable length.
#define UDP_PURE_MSG_HDR_LEN(hdr) (sizeof(hdr) - sizeof(nrf51_udp_proto_hdr))

#define UDP_SEND_ID_MSG_SZ UDP_PURE_MSG_HDR_LEN(nrf51_udp_send_id_hdr)

#define UDP_GPIO_MSG_SZ UDP_PURE_MSG_HDR_LEN(nrf51_udp_gpio_hdr)

typedef void (*nrf51_udp_packet_handler)(void* opaque, uint8_t* data, uint_fast16_t len);

static inline uint_fast16_t udp_read_len(nrf51_udp_proto_hdr * hdr)
{
    uint_fast16_t ret = hdr->lenHi << 8 | hdr->lenLow;
    return ret;
}

static inline void udp_set_len(nrf51_udp_proto_hdr * hdr, uint_fast16_t len)
{
    hdr->lenHi = len >> 8;
    hdr->lenLow = len & 0xff;
}

static inline void udp_set_uint16(uint8_t * dest, uint16_t value)
{
    dest[0] = value >> 8;
    dest[1] = value & 0xff;
}

static inline void hexdump(const char * label, const void * data, const int size)
{
    printf("%s:", label);
    const uint8_t * _data = data;
    for (int i = 0; i < size; i++)
    {
        if (i % 4 == 0)
        {
            puts("");
        }
        printf("%02x ", _data[i]);
    }
    puts("");
}

/*************************************\
 * Shared Functions
\*************************************/
//nrf51_uart_comm
int nrf51_uart_comm_init(void);
int nrf51_uart_comm_read(char * pchBuf, int nAvailable);
int nrf51_uart_comm_write(char * pchBuf, int nLen);

int nrf51_udp_send(void * data, size_t len);
void nrf51_udp_fill_hdr(nrf51_udp_proto_hdr *hdr, uint8_t proto_type, uint16_t len);
void _nrf51_trigger_hardfault(const char * file, int line);
#define nrf51_trigger_hardfault() _nrf51_trigger_hardfault(__FILE__, __LINE__)
void nrf51_radio_event_ready_ppi(void);

extern uint32_t nrf_id;

#endif
