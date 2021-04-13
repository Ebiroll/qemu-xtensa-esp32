#ifndef __NRF51_RADIO_H__
#define __NRF51_RADIO_H__

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "hw/boards.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/arm/arm.h"
#include "sysemu/sysemu.h"

#if 0
#define RDP(fmt, ...) do{\
    qemu_mutex_lock(&mtx_console);\
    printf("[RADIO] ");\
    printf(fmt, __VA_ARGS__);\
    puts("");\
    fflush(stdout);\
    qemu_mutex_unlock(&mtx_console);\
    }while(0)
#else
#define RDP(fmt, ...)do{}while(0)
#endif

#define RAD_LOCK() do{ \
/*fprintf(stderr, "RAD_LOCK: %s\n", __func__); fflush(stderr);*/\
qemu_mutex_lock(&mtx_radio); \
}while(0)

#define RAD_UNLOCK() do{ \
/*fprintf(stderr, "RAD_UNLOCK: %s\n", __func__); fflush(stderr);*/\
qemu_mutex_unlock(&mtx_radio); \
}while(0)

#define RAD_UNLOCK_RET(val) do{\
    volatile typeof(val) ret = val;\
    RAD_UNLOCK();\
    return ret;\
}while(0)

#define RDP_DUMP(var) RDP(#var ": %u", var)

#define PRINT_INCORRECT_STATE() do{\
    RDP("%s:%d incorrect radio state: %s, task: %s", \
    __FILE__, __LINE__, \
    state_to_str(s->radio_state), task_to_str(s->radio_task)); \
    g_assert(!"EXIT!"); \
} while(0)

//TODO: rename this macros
#define READ_MSB_BYTE(val, byte) ( ( (val) >> (byte)*8) & 0xff )

#define PBUF_SZ 512
#define AIR_PACKET_HDR_SZ (2u)

/* PCNF1 Macros */
#define PCNF1_MAXLEN_READ(reg)  ( (reg) & 0xff )
#define PCNF1_WHITEEN_READ(reg) ( ((reg) >> 25) & 0x1 )

/* SHORTS Macros */
#define SHORTS_READY_START(reg)         GET_BIT((reg), 0)
#define SHORTS_END_DISABLE(reg)         GET_BIT((reg), 1)
#define SHORTS_DISABLED_TXEN(reg)       GET_BIT((reg), 2)
#define SHORTS_DISABLED_RXEN(reg)       GET_BIT((reg), 3)
#define SHORTS_ADDRESS_RSSISTART(reg)   GET_BIT((reg), 4)
#define SHORTS_END_START(reg)           GET_BIT((reg), 5)
#define SHORTS_ADDRESS_BCSTART(reg)     GET_BIT((reg), 6)
#define SHORTS_DISABLED_RSSISTOP(reg)   GET_BIT((reg), 8)

/* Interrupts */
#define RADIO_INTEN_READY    (1<<0)
#define RADIO_INTEN_ADDRESS  (1<<1)
#define RADIO_INTEN_PAYLOAD  (1<<2)
#define RADIO_INTEN_END      (1<<3)
#define RADIO_INTEN_DISABLED (1<<4)
#define RADIO_INTEN_DEVMATCH (1<<5)
#define RADIO_INTEN_DEVMISS  (1<<6)
#define RADIO_INTEN_RSSIEND  (1<<7)
#define RADIO_INTEN_BCMATCH  (1<<10)

enum
{
    //Tasks
    O_RADIO_TXEN = 0x0,
    O_RADIO_RXEN = 0x4,
    O_RADIO_START = 0x8,
    O_RADIO_STOP = 0xC,
    O_RADIO_DISABLE = 0x10,
    O_RADIO_RSSISTART = 0x14,
    O_RADIO_RSSISTOP = 0x18,
    O_RADIO_BCSTART = 0x1C,
    O_RADIO_BCSTOP = 0x20,

    //Events
    O_RADIO_READY = 0x100,
    O_RADIO_ADDRESS = 0x104,
    O_RADIO_PAYLOAD = 0x108,
    O_RADIO_END = 0x10C,
    O_RADIO_DISABLED = 0x110,
    O_RADIO_DEVMATCH = 0x114,
    O_RADIO_DEVMISS = 0x118,
    O_RADIO_RSSIEND = 0x11C,
    O_RADIO_BCMATCH = 0x128,

    //Registers
    O_RADIO_SHORTS = 0x200,
    O_RADIO_INTENSET = 0x304,
    O_RADIO_INTENCLR = 0x308,
    O_RADIO_CRCSTATUS = 0x400,
    O_RADIO_RXMATCH = 0x408,
    O_RADIO_RXCRC = 0x40C,
    O_RADIO_DAI = 0x410,
    O_RADIO_PACKETPTR = 0x504,
    O_RADIO_FREQUENCY = 0x508,
    O_RADIO_TXPOWER = 0x50C,
    O_RADIO_MODE = 0x510,
    O_RADIO_PCNF0 = 0x514,
    O_RADIO_PCNF1 = 0x518,
    O_RADIO_BASE0 = 0x51C,
    O_RADIO_BASE1 = 0x520,
    O_RADIO_PREFIX0 = 0x524,
    O_RADIO_PREFIX1 = 0x528,
    O_RADIO_TXADDRESS = 0x52C,
    O_RADIO_RXADDRESSES = 0x530,
    O_RADIO_CRCCNF = 0x534,
    O_RADIO_CRCPOLY = 0x538,
    O_RADIO_CRCINIT = 0x53C,
    O_RADIO_TEST = 0x540,
    O_RADIO_TIFS = 0x544,
    O_RADIO_RSSISAMPLE = 0x548,
    O_RADIO_STATE = 0x550,
    O_RADIO_DATAWHITEIV = 0x554,
    O_RADIO_BCC = 0x560,
    O_RADIO_DAB0 = 0X600,
    O_RADIO_DAB1 = 0X604,
    O_RADIO_DAB2 = 0X608,
    O_RADIO_DAB3 = 0X60C,
    O_RADIO_DAB4 = 0X610,
    O_RADIO_DAB5 = 0X614,
    O_RADIO_DAB6 = 0X618,
    O_RADIO_DAB7 = 0X61C,

    O_RADIO_DAP0 = 0X620,
    O_RADIO_DAP1 = 0X624,
    O_RADIO_DAP2 = 0X628,
    O_RADIO_DAP3 = 0X62C,
    O_RADIO_DAP4 = 0X630,
    O_RADIO_DAP5 = 0X634,
    O_RADIO_DAP6 = 0X648,
    O_RADIO_DAP7 = 0X63C,

    O_RADIO_DACNF = 0x640,

    O_RADIO_OVERRIDE0 = 0x724,
    O_RADIO_OVERRIDE2 = 0x728,
    O_RADIO_OVERRIDE3 = 0x72C,
    O_RADIO_OVERRIDE4 = 0x730,
    O_RADIO_OVERRIDE5 = 0x734,

    O_RADIO_POWER = 0xFFC,
};

enum
{
    enmAirTypeJoin = 0x00,
    enmAirTypeData = 0x01
};

typedef enum
{
    enmEndianLittle = 0,
    enmEndianBig = 1
}NRF51_ENDIAN;

typedef enum
{
    enmNrf1Mbit   = 0,
    enmNrf2Mbit   = 1,
    enmNrf250Kbit = 2,
    Ble_1Mbit     = 4
}NRF51_MODE;

typedef enum
{
    enmStateDisabled  = 0, //Must always be zero.
    enmStateRxRu      = 1,
    enmStateRxIdle    = 2,
    enmStateRx        = 3,
    enmStateRxDisable = 4,
    enmStateTxRu      = 9,
    enmStateTxIdle    = 10,
    enmStateTx        = 11,
    enmStateTxDisable = 12,
}NRF51_RADIO_STATE;

typedef enum
{
    enmTaskNone = 0, //Must always be zero.
    enmTaskTxEn,
    enmTaskRxEn,
    enmTaskStart,
    enmTaskStop,
    enmTaskDisable
}radio_task_t;

typedef struct
{
    bool WHITEEN;//Packet whitening enable
    uint8_t ENDIAN;
    uint8_t BALEN; //Base address length
    uint8_t STATLEN;
    uint8_t MAXLEN;
}NRF51_RADIO_PCNF1;

typedef struct
{
    uint8_t LFLEN; //Number of bits
    uint8_t S0LEN; //Number of bytes (here max is 1 byte)
    uint8_t S1LEN; //Number of bits
}NRF51_RADIO_PCNF0;

typedef struct
{
    uint8_t uLen;
    bool bSkipAddr;
}NRF51_RADIO_CRCCNF;

typedef struct _nrf51_radio_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    QEMUTimer * qtimer;
    bool timer_running;

    //Fields below will be set to zero on reset.
    //radio_state must always be at the beginning and REG must be the last.
    NRF51_RADIO_STATE radio_state;
    radio_task_t radio_task;
    radio_task_t next_on_disabled;
    bool irq17_pending;

    bool bPacketReceived;
    uint8_t rx_buf[512];
    uint8_t rx_len;

    struct
    {
        uint8_t uFrequency; //Freq = 2400 + value
        NRF51_RADIO_PCNF0 PCNF0;
        NRF51_RADIO_PCNF1 PCNF1;
        NRF51_RADIO_CRCCNF CRCCNF;
        uint32_t PACKETPTR;
        uint32_t CRCINIT;
        uint32_t CRCPOLY;
        uint8_t AP[8]; //PREFIX0[0..3] + PREFIX1[4..7]
        uint32_t BASE[2]; //BASE0 & BASE1
        uint8_t uTxAddrSelect;
        bool RXADDRESSES_ADDR[8];
        uint8_t uMode;
    }ActiveConf;

    //ptimer_state * pt_conversion;

    //REG struct must always be the last field in radio state.
    struct
    {
        //Events
        uint32_t READY;
        uint32_t ADDRESS;
        uint32_t PAYLOAD;
        uint32_t END;
        uint32_t DISABLED;
        uint32_t DEVMATCH;
        uint32_t DEVMISS;
        uint32_t RSSIEND;
        uint32_t BCMATCH;

        //Registers
        uint32_t SHORTS;
        //TODO: No mention of INTEN register in reference manual. Check.
        uint32_t INTEN; //Not a real register, used internally here
        uint32_t CRCSTATUS;
        uint32_t RXMATCH;
        uint32_t RXCRC;
        uint32_t DAI;
        uint32_t PACKETPTR;
        uint32_t FREQUENCY;
        uint32_t TXPOWER;
        uint32_t MODE;
        uint32_t PCNF0;
        uint32_t PCNF1;
        uint32_t BASE0;
        uint32_t BASE1;
        uint32_t PREFIX0;
        uint32_t PREFIX1;
        uint32_t TXADDRESS;
        uint32_t RXADDRESSES;
        uint32_t CRCCNF;
        uint32_t CRCPOLY;
        uint32_t CRCINIT;
        uint32_t TEST;
        uint32_t TIFS; //Only available in BLE_1MBIT mode.
        uint32_t RSSISAMPLE;
        //uint32_t STATE;
        uint32_t DATAWHITEIV;
        uint32_t BCC;
        uint32_t DAB[8];
        uint32_t DAP[8];
        uint32_t DACNF;
        uint32_t OVERRIDE[5];
        uint32_t POWER;

    } REG;
} nrf51_radio_state;

/* only byte variables are used for simple alignment */
typedef struct /* __attribute__((packed)) */ //packing not required.
{
    nrf51_udp_proto_hdr proto_hdr;
    uint8_t type;
    uint8_t mode;
    uint8_t data_start[1];
} nrf51_air_packet;

typedef struct
{
    nrf51_udp_proto_hdr proto_hdr;
    uint8_t type;
    uint8_t mode;
    uint8_t buffer[PBUF_SZ];
} nrf51_air_packet_buff;

//Make sure that structure alignment is as expected.
QEMU_BUILD_BUG_ON(sizeof(nrf51_air_packet) != PROTO_HDR_SZ + AIR_PACKET_HDR_SZ + 1);

uint64_t nrf51_radio_read(void *opaque, hwaddr offset,
                          unsigned size);
void nrf51_radio_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size);
void nrf51_radio_timer(void *opaque);
void nrf51_radio_udp_init(nrf51_radio_state *s);
void nrf51_radio_do_rx(void * opaque, uint8_t * data, uint_fast16_t len);
#endif
