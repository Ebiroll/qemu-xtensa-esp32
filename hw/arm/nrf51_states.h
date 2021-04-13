#ifndef __NRF51_PRIV_H__
#define __NRF51_PRIV_H__
#include "nrf51.h"
#include "crypto/cipher.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#endif

#define AES_ECB_BLOCK_SZ  (16) //Unit is bytes
#define AES_ECB_HDR_SZ    (AES_ECB_BLOCK_SZ*3)
#define RTC_NUM_CC_REGS   (4)
#define TIMER_NUM_CC_REGS (4)
#define NRF51_CODESIZE    (256)
#define PPI_NUM_CHG (4)
#define PPI_NUM_USER_CHANS (16)
#define PPI_NUM_FIXED_CHANS (12)
#define AES_CCM_NONCE_SIZE (13)

#pragma region DeviceStates
/*************************************\
 * Device States
\*************************************/
typedef struct _nrf51_rng_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state * ptimer;
    struct
    {
        uint32_t VALRDY;
        uint32_t SHORTS;
        uint32_t INTEN;
        uint32_t CONFIG;
        uint32_t VALUE;
    } REG;
} nrf51_rng_state;

typedef struct _nrf51_gpio_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;

    struct _nrf51_gpte_state * owner[NUM_GPIO_PINS];
    int owner_id[NUM_GPIO_PINS];
    struct {
        uint32_t OUT;       //0x504
        uint32_t OUTSET;    //0x508
        uint32_t OUTCLR;    //0x50C
        uint32_t IN;
        uint32_t DIR;
        uint32_t DIRSET;
        uint32_t DIRCLR;
        uint32_t PINCNF[NUM_GPIO_PINS];
    } REG;
} nrf51_gpio_state;

typedef struct _nrf51_gpte_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;

    //Can be used for in/out, depending on configured mode.
    uint8_t io_state[4];
    struct
    {
        //Events
        uint32_t IN[4];
        uint32_t PORT;

        //Registers
        uint32_t INTEN;
        uint32_t CONFIG[4];
    } REG;
} nrf51_gpte_state;

typedef struct _nrf51_clock_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    struct
    {
        //Events
        uint32_t HFCLKSTARTED;
        uint32_t LFCLKSTARTED;
        uint32_t DONE;
        uint32_t CTTO;

        //Registers
        uint32_t INTEN;
        uint32_t HFCLKRUN;
        uint32_t HFCLKSTAT;
        uint32_t LFCLKRUN;
        uint32_t LFCLKSTAT;
        uint32_t LFCLKSRCCOPY;
        uint32_t LFCLKSRC;
        uint32_t CTIV;
        uint32_t XTALFREQ;
    } REG;
} nrf51_clock_state;

typedef struct
{
    uint8_t key[AES_ECB_BLOCK_SZ];
    uint8_t cleartext[AES_ECB_BLOCK_SZ];
    uint8_t ciphertext[AES_ECB_BLOCK_SZ];
} nrf51_ecb_data;

//Make sure that struct is not aligned in any way.
QEMU_BUILD_BUG_ON(sizeof(nrf51_ecb_data) !=  AES_ECB_HDR_SZ);

typedef struct _nrf51_ecb_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    struct
    {
        //Events
        uint32_t ENDECB;
        uint32_t ERRORECB;

        //Registers
        uint32_t INTEN;
        uint32_t ECBDATAPTR;
    } REG;

    QCryptoCipher * cipher_ctx;
    uint8_t current_key[AES_ECB_BLOCK_SZ];
    nrf51_ecb_data ecb_data;
} nrf51_ecb_state;

typedef struct _nrf51_uart_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state * ptimer;

    int bUartEnabled;
    int bFlowCtrlEnabled; //HW Flow Control
    int bCtsEnabled;
    int bNotCtsEnabled;
    int bNewByte;
    int bReadOk;
    uint32_t uTimerTaskFlags;

    struct {
        //Events
        uint32_t CTS;
        uint32_t NCTS;
        uint32_t RXDRDY;
        uint32_t TXDRDY;
        uint32_t ERROR;
        uint32_t RXTO; //RX Timeout

        //Registers
        uint32_t INTEN;
        uint32_t ENABLE;
        uint32_t PSELCTS;
        uint32_t PSELRTS;
        uint32_t PSELRXD;
        uint32_t PSELTXD;
        uint32_t RXD;
        uint32_t TXD;
        uint32_t BAUDRATE;
        uint32_t CONFIG;
    } REG;
} nrf51_uart_state;

typedef struct _nrf51_rtc_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    uint32_t base;
    qemu_irq irq;
    QEMUTimer * qtimer;
    int num_instance;
    bool bRunning;
    QemuMutex mtx;
    int64_t rtc_begin_ns;
    double ns_per_tick;
    uint32_t next_tick;
    struct {
        //Tasks

        //Events
        uint32_t TICK;
        uint32_t OVRFLW;
        uint32_t COMPARE[RTC_NUM_CC_REGS]; //4 Events
        //Registers
        uint32_t INTEN;
        uint32_t EVTEN;
        uint32_t COUNTER;
        uint32_t PRESCALER;
        uint32_t CC[RTC_NUM_CC_REGS]; // 4 Compare registers
    } REG;
} nrf51_rtc_state;

typedef struct _nrf51_adc_state {
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    qemu_irq irq;
    ptimer_state * pt_conversion;
    int bConversionActive;
    int nNoiseBits;
    struct {
        uint32_t START;
        uint32_t STOP;
        uint32_t END;
        uint32_t INTEN;
        uint32_t ENABLE;
        uint32_t CONFIG;
        uint32_t RESULT;
    } REG;
} nrf51_adc_state;

typedef struct _nrf51_timer_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    uint32_t base;
    uint32_t counter_max;
    int num_instance;
    qemu_irq irq;
    bool task_started;
    QEMUTimer * qtimer;
    uint32_t ns_per_tick;
    uint32_t cc_used; //CC registers used to setup the qemu timer.
    int64_t clock_begin;
    uint32_t tick;
    struct
    {
        //Events
        uint8_t COMPARE[TIMER_NUM_CC_REGS];
        //Registers
        uint32_t INTEN;
        uint16_t SHORTS;
        uint8_t  MODE;
        uint8_t  BITMODE;
        uint8_t  PRESCALER;
        uint32_t CC[TIMER_NUM_CC_REGS];
    } REG;
} nrf51_timer_state;

typedef struct _nrf51_nvm_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    MemoryRegion flash;
    int64_t opstart;
    ptimer_state * file_sync;
    FILE * fpkernel;
    uint8_t nvm_dirty_bits[NRF51_CODESIZE/8]; //A bit or each page
    struct
    {
        uint32_t CONFIG;
    } REG;
} nrf51_nvm_state;

typedef struct _nrf51_ficr_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    struct
    {

    } REG;
} nrf51_ficr_state;

typedef struct _nrf51_uicr_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    struct
    {
        uint32_t CLENR0;
        uint32_t XTALFREQ;
        uint32_t BOOTLOADERADDR;
        uint32_t CUSTOMER[32];
    } REG;
} nrf51_uicr_state;

// 32 Customer Registers + 3 device registers.
QEMU_BUILD_BUG_ON(sizeof(nrf51_uicr_state) ==  sizeof(uint32_t) * (32 + 3));

typedef struct _nrf51_ccm_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    struct
    {
        uint8_t ENDKSGEN;
        uint8_t ENDCRYPT;
        uint8_t ERROR;
        uint8_t INTEN;
        uint8_t SHORTS;
        uint8_t MICSTATUS;
        uint8_t ENABLE;
        uint8_t MODE;
        uint32_t CNFPTR;
        uint32_t INPTR;
        uint32_t OUTPTR;
        uint32_t SCRATCHPTR;
    } REG;
    uint8_t nonce[AES_CCM_NONCE_SIZE];
    uint8_t key[AES_ECB_BLOCK_SZ]; //TODO: Use different define
} nrf51_ccm_state;

typedef struct _nrf51_wdt_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    bool started;
    struct
    {
        uint8_t INTEN : 1;
        uint8_t RUNSTATUS : 1;
        uint8_t REQSTATUS;
        uint8_t RREN;
        uint8_t CONFIG;
        uint32_t CRV;
    } REG;
} nrf51_wdt_state;

typedef struct _nrf51_ppi_state
{
    SysBusDevice sb_parent;
    MemoryRegion iomem;
    /*
     * PPI uses a global variable for state information.
     * It is used by other peripherals.
     */
} nrf51_ppi_state;

typedef struct
{
    uint32_t addr;
    int ref_cnt;
    //An event can exist maximum in n channels where n = (PPI_NUM_USER_CHANS + PPI_NUM_FIXED_CHANS).
    uint8_t channels[PPI_NUM_USER_CHANS + PPI_NUM_FIXED_CHANS];
} ppi_event_info;

typedef struct
{
    struct qht * event_map;
    struct
    {
        uint32_t CHEN; //Channel Enable
        struct
        {
            uint32_t EEP; //Event end point
            uint32_t TEP; //Task end point
        } CH[32];
        uint32_t CHG[PPI_NUM_CHG]; //Channel group configuration.
    } REG;

} ppi_global_state;

#pragma endregion

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#endif
