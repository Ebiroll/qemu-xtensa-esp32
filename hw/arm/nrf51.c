#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "qemu-common.h"
#include "cpu.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "hw/misc/unimp.h"
#include "hw/ptimer.h"
#include "sysemu/sysemu.h"
#include "nrf51_states.h"
#include "nrf51_radio.h"
#include "nrf51_helper.h"
#include "sys/socket.h"
#include "crypto/cipher.h"
#include "qemu/qht.h" //hashtable
#include "nrf51_aes_ccm.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#endif

/*************************************\
 * Defines that are required here.
\*************************************/
#define AES_ECB_READ_SZ (AES_ECB_BLOCK_SZ*2)
#define AES_ECB_CIPHERTEXT_OFFSET (AES_ECB_BLOCK_SZ*2)
#define NRF51_CODEPAGESIZE (1024)
#define NRF51_FLASH_SIZE_IN_BYTES (NRF51_CODESIZE * NRF51_CODEPAGESIZE) //256K
#define AES_CCM_MAX_PKT_SZ (27) //FIXME: Need to verify on the hardware.
#define AES_CCM_SCRATCH_SZ (16 + AES_CCM_MAX_PKT_SZ)

#pragma region TypeDevInitIO
/*************************************\
 * Device Types
\*************************************/
static const char TYPE_NRF51_ADC[] = "nrf51-adc";
static const char TYPE_NRF51_CLOCK[] = "nrf51-clock";
static const char TYPE_NRF51_ECB[] = "nrf51-ecb";
static const char TYPE_NRF51_GPIO[] = "nrf51-gpio";
static const char TYPE_NRF51_GPTE[] = "nrf51-gpte";
static const char TYPE_NRF51_RADIO[] = "nrf51-radio";
static const char TYPE_NRF51_RNG[] = "nrf51-rng";
static const char TYPE_NRF51_RTC[] = "nrf51-rtc";
static const char TYPE_NRF51_TIMER[] = "nrf51-timer";
static const char TYPE_NRF51_UART[] = "nrf51-uart";
static const char TYPE_NRF51_NVM[] = "nrf51-nvm";
static const char TYPE_NRF51_FICR[] = "nrf51-ficr";
static const char TYPE_NRF51_UICR[] = "nrf51-uicr";
static const char TYPE_NRF51_WDT[] = "nrf51-wdt";
static const char TYPE_NRF51_PPI[] = "nrf51-ppi";
static const char TYPE_NRF51_CCM[] = "nrf51-ccm";

/*************************************\
 * Dev. Init & IO Function Prototypes.
\*************************************/
static void nrf51_adc_init(Object *obj);
static void nrf51_clock_init(Object *obj);
static void nrf51_ecb_init(Object *obj);
static void nrf51_gpio_init(Object *obj);
static void nrf51_gpte_init(Object *obj);
static void nrf51_radio_init(Object *obj);
static void nrf51_rng_init(Object *obj);
static void nrf51_rtc_init(Object *obj);
static void nrf51_timer_init(Object *obj);
static void nrf51_uart_init(Object *obj);
static void nrf51_nvm_init(Object *obj);
static void nrf51_ficr_init(Object *obj);
static void nrf51_uicr_init(Object *obj);
static void nrf51_wdt_init(Object *obj);
static void nrf51_ppi_init(Object *obj);
static void nrf51_ccm_init(Object *obj);

static void nrf51_adc_class_init(ObjectClass *class, void *data);
static void nrf51_clock_class_init(ObjectClass *class, void *data);
static void nrf51_ecb_class_init(ObjectClass *class, void *data);
static void nrf51_gpio_class_init(ObjectClass *class, void *data);
static void nrf51_gpte_class_init(ObjectClass *class, void *data);
static void nrf51_radio_class_init(ObjectClass *class, void *data);
static void nrf51_rng_class_init(ObjectClass *class, void *data);
static void nrf51_rtc_class_init(ObjectClass *class, void *data);
static void nrf51_timer_class_init(ObjectClass *class, void *data);
static void nrf51_uart_class_init(ObjectClass *class, void *data);
static void nrf51_nvm_class_init(ObjectClass *class, void *data);
static void nrf51_ficr_class_init(ObjectClass *class, void *data);
static void nrf51_uicr_class_init(ObjectClass *class, void *data);
static void nrf51_wdt_class_init(ObjectClass *class, void *data);
static void nrf51_ppi_class_init(ObjectClass *class, void *data);
static void nrf51_ccm_class_init(ObjectClass *class, void *data);

static void nrf51_gpio_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_clock_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_gpte_write(void *opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_rtc_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_adc_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_uart_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_ecb_write(void *opaque, hwaddr offset, uint64_t value, unsigned size);
static void nrf51_rng_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_timer_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_nvm_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_nvm_data_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_ficr_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_uicr_write(void *opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_wdt_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_ppi_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);
static void nrf51_ccm_write(void * opaque, hwaddr offset, uint64_t value, unsigned int size);

static uint64_t nrf51_adc_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_clock_read(void * opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_ecb_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_gpio_read(void * opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_gpte_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_rng_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_rtc_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_timer_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_uart_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_nvm_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_ficr_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_uicr_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_wdt_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_ppi_read(void *opaque, hwaddr offset, unsigned int size);
static uint64_t nrf51_ccm_read(void *opaque, hwaddr offset, unsigned int size);

#pragma endregion

#pragma region TypeInfo
/*************************************\
 * TypeInfo
\*************************************/
static const TypeInfo nrf51_mod_gpio = {
    .name          = TYPE_NRF51_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_gpio_state),
    .instance_init = nrf51_gpio_init,
    .class_init    = nrf51_gpio_class_init,
};

static const TypeInfo nrf51_mod_gpte = {
    .name          = TYPE_NRF51_GPTE,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_gpte_state),
    .instance_init = nrf51_gpte_init,
    .class_init    = nrf51_gpte_class_init,
};

static const TypeInfo nrf51_mod_clock = {
    .name          = TYPE_NRF51_CLOCK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_clock_state),
    .instance_init = nrf51_clock_init,
    .class_init    = nrf51_clock_class_init,
};

static const TypeInfo nrf51_mod_rtc = {
    .name          = TYPE_NRF51_RTC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_rtc_state),
    .instance_init = nrf51_rtc_init,
    .class_init    = nrf51_rtc_class_init,
};

static const TypeInfo nrf51_mod_ecb = {
    .name          = TYPE_NRF51_ECB,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_ecb_state),
    .instance_init = nrf51_ecb_init,
    .class_init    = nrf51_ecb_class_init,
};

static const TypeInfo nrf51_mod_adc = {
    .name          = TYPE_NRF51_ADC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_adc_state),
    .instance_init = nrf51_adc_init,
    .class_init    = nrf51_adc_class_init,
};

static const TypeInfo nrf51_mod_uart = {
    .name          = TYPE_NRF51_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_uart_state),
    .instance_init = nrf51_uart_init,
    .class_init    = nrf51_uart_class_init,
};

static const TypeInfo nrf51_mod_radio = {
    .name          = TYPE_NRF51_RADIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_radio_state),
    .instance_init = nrf51_radio_init,
    .class_init    = nrf51_radio_class_init,
};

static const TypeInfo nrf51_mod_timer = {
    .name          = TYPE_NRF51_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_timer_state),
    .instance_init = nrf51_timer_init,
    .class_init    = nrf51_timer_class_init,
};

static const TypeInfo nrf51_mod_rng = {
    .name          = TYPE_NRF51_RNG,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_rng_state),
    .instance_init = nrf51_rng_init,
    .class_init    = nrf51_rng_class_init,
};

static const TypeInfo nrf51_mod_nvm = {
    .name          = TYPE_NRF51_NVM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_nvm_state),
    .instance_init = nrf51_nvm_init,
    .class_init    = nrf51_nvm_class_init,
};

static const TypeInfo nrf51_mod_ficr = {
    .name          = TYPE_NRF51_FICR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_ficr_state),
    .instance_init = nrf51_ficr_init,
    .class_init    = nrf51_ficr_class_init,
};

static const TypeInfo nrf51_mod_uicr = {
    .name          = TYPE_NRF51_UICR,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_uicr_state),
    .instance_init = nrf51_uicr_init,
    .class_init    = nrf51_uicr_class_init,
};

static const TypeInfo nrf51_mod_wdt = {
    .name          = TYPE_NRF51_WDT,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_wdt_state),
    .instance_init = nrf51_wdt_init,
    .class_init    = nrf51_wdt_class_init,
};

static const TypeInfo nrf51_mod_ppi = {
    .name          = TYPE_NRF51_PPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_ppi_state),
    .instance_init = nrf51_ppi_init,
    .class_init    = nrf51_ppi_class_init,
};

static const TypeInfo nrf51_mod_ccm = {
    .name          = TYPE_NRF51_CCM,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(nrf51_ccm_state),
    .instance_init = nrf51_ccm_init,
    .class_init    = nrf51_ccm_class_init,
};

#pragma endregion //TypeInfo

#pragma region MemoryRegionOps
/*************************************\
 * MemoryRegionOps
\*************************************/
static const MemoryRegionOps nrf51_gpio_ops = {
    .read = nrf51_gpio_read,
    .write = nrf51_gpio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_rtc_ops = {
    .read = nrf51_rtc_read,
    .write = nrf51_rtc_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_adc_ops = {
    .read = nrf51_adc_read,
    .write = nrf51_adc_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_uart_ops = {
    .read = nrf51_uart_read,
    .write = nrf51_uart_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_clock_ops = {
    .read = nrf51_clock_read,
    .write = nrf51_clock_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_ecb_ops = {
    .read = nrf51_ecb_read,
    .write = nrf51_ecb_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_radio_ops = {
    .read = nrf51_radio_read,
    .write = nrf51_radio_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_timer_ops = {
    .read = nrf51_timer_read,
    .write = nrf51_timer_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_rng_ops = {
    .read = nrf51_rng_read,
    .write = nrf51_rng_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_gpte_ops = {
    .read = nrf51_gpte_read,
    .write = nrf51_gpte_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_nvm_ops = {
    .read = nrf51_nvm_read,
    .write = nrf51_nvm_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_nvm_data_ops = {
    .read = NULL,
    .write = nrf51_nvm_data_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_ficr_ops = {
    .read = nrf51_ficr_read,
    .write = nrf51_ficr_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_uicr_ops = {
    .read = nrf51_uicr_read,
    .write = nrf51_uicr_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_wdt_ops = {
    .read = nrf51_wdt_read,
    .write = nrf51_wdt_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_ppi_ops = {
    .read = nrf51_ppi_read,
    .write = nrf51_ppi_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps nrf51_ccm_ops = {
    .read = nrf51_ccm_read,
    .write = nrf51_ccm_write,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

#pragma endregion //MemoryRegionOps

//TODO: Use the correct value, check datasheet?
#define NUM_IRQ_LINES (64)
#define FREQ_16MHZ (16000000)

#define UART_ENABLE_REG_VAL  0x4
#define UART_DISABLE_REG_VAL 0x0

#define NVM_CONFIG_RO  0x0  //Read-only
#define NVM_CONFIG_WEN 0x1  //Write Enable
#define NVM_CONFIG_EEN 0x2  //Erase Enable

#define TIMER_MODE_TIMER   0x0
#define TIMER_MODE_COUNTER 0x1

#define CLOCK_STAT_RUNNING (1<<16)

#define WDT_RELOAD_VALUE (0x6E524635UL)
#define ECB_ERRORECB_MASK (0x2)
#define ECB_ENDECB_MASK (0x1)

//Simulated process time for NVM actions.
#define NVM_OP_TIME_NS (300)

#define NRF51_FLASH_ORIGIN (0x0)

#define SERVER_PORT 5151
#define SERVER_ADDR "127.0.0.1"

#define UICR_REG_FILE "uicr.bin"

typedef struct
{
    hwaddr base;
    int irq;
} base_irq_pair;

typedef struct
{
    hwaddr base;
    int irq;
    uint32_t counter_max; // Maximum counter size
} timer_config;

typedef struct _nrf51_udp_conn
{
    int fd;
    struct sockaddr_in si_server;
    bool bCanWrite;
    uint_fast32_t uBytesRcvd;
    uint_fast32_t uBytesSent;
} nrf51_udp_conn;

typedef struct
{
    const char * kernel_filename;
    //FIXME: Use nvic
    DeviceState * nvic;
    MemoryRegion sram;
} nrf51_boot_info;

typedef struct ccm_cnf
{
    uint8_t key[AES_ECB_BLOCK_SZ]; //TODO: Use a new define as s->key[]
    //pktr[4]: Bit0-6: used, Bit7: ignored
    uint64_t pktctr; //big endian 39-bit
    uint8_t direction_bit; //bit0: direction, zero padded
    uint8_t iv[8];
} QEMU_PACKED ccm_cnf_t;
QEMU_BUILD_BUG_ON(sizeof(ccm_cnf_t) != 33);

#define CCM_PKT_CTR_MASK ((1 << 39) - 1)

//Common functions for encrypted and plain text packet structures.
static uint8_t get_ccm_hdr(uint8_t * ptr) { return *ptr; }
static uint8_t get_ccm_len(uint8_t * ptr) { return ptr[1]; }
static uint8_t * get_ccm_data_ptr(uint8_t * ptr) { return ptr + 3; }

#pragma region Implementation
static void nrf51_udp_send_id_handler(void * opaque, uint8_t * data, uint_fast16_t len);
static void nrf51_gpio_udp_handler(void * opaque, uint8_t * data, uint_fast16_t len);
static void nrf51_timer_tick(void * opaque);
static void nrf51_timer_qtick(void * opaque);
static void nrf51_rtc_timer(void * opaque);
static void nrf51_rtc_tick(void *opaque);

static int64_t nrf51_timer_set_next_event(nrf51_timer_state * s);

static uint32_t nrf51_timer_getmask(nrf51_timer_state * s);

static void nrf51_nvm_erase_page(nrf51_nvm_state * s, uint32_t offset, bool is_region_0);
static void nrf51_nvm_set_dirty_bit(nrf51_nvm_state * s, uint_fast32_t page_num);
static void nrf51_uicr_load_regs(nrf51_uicr_state * s);
static void nrf51_uicr_save_regs(nrf51_uicr_state * s);
static void ppi_event_filter(uint32_t event_addr);
static void ppi_add_event(uint8_t chan, uint32_t eep, uint32_t tep);

static inline int nrf51_gpte_read_mode(nrf51_gpte_state *s, const int id)
{
    return s->REG.CONFIG[id] & 0x3;
}

static inline int nrf51_gpte_read_psel(nrf51_gpte_state *s, const int id)
{
    return (s->REG.CONFIG[id] >> 8) & 0x1f;
}

static inline int nrf51_gpte_read_polarity(nrf51_gpte_state *s, const int id)
{
    return (s->REG.CONFIG[id] >> 16) & 0x3;
}

static void nrf51_gpte_set_event(nrf51_gpte_state *s, const int id, const bool state)
{
    int polarity = nrf51_gpte_read_polarity(s, id);
    printf("GPIOTE event, id: %d\n", id);

    switch(polarity)
    {
        case GPTE_CONFIG_NONE:
            //No event generated.
            return;

        case GPTE_CONFIG_RISING:
            if (!state){
                //No event.
                return;
            }

            if (s->io_state[id]){
                //Already HIGH
                return;
            }
            break;

        case GPTE_CONFIG_FALLING:
            if(state){
                //No event.
                return;
            }

            if(!s->io_state[id]){
                //Already LOW
                return;
            }

        case GPTE_CONFIG_TOGGLE:
            //Let it generate event for any change.
            break;
    }

    s->io_state[id] = state;
    //FIXME: Check PORT event.
    s->REG.IN[id] = 1;
    ppi_event_filter(GPTE_BASE + O_GPTE_IN0 + id * 4);
    if (s->REG.INTEN & (1<<id)){
        qemu_irq_pulse(s->irq);
    }
}

static const base_irq_pair RTC_BASE_IRQ_LIST[RTC_TOTAL] = {
    { RTC0_BASE, IRQ_RTC0 },
    { RTC1_BASE, IRQ_RTC1 }
    //{ RTC2_BASE, IRQ_RTC2 } //Doesn't exist on NRF51
};

static const timer_config TIMER_CONFIG[TIMER_TOTAL] = {
    { TIMER0_BASE, IRQ_TIMER0, 0xFFFFFFFF}, //32 bit counter
    { TIMER1_BASE, IRQ_TIMER1, 0xFFFF},     //16 bit counter
    { TIMER2_BASE, IRQ_TIMER2, 0xFFFF}      //16 bit counter
};

nrf51_udp_packet_handler packet_handlers[PROTO_TOTAL] =
{
    [PROTO_SEND_ID] = nrf51_udp_send_id_handler,
    [PROTO_RADIO] = nrf51_radio_do_rx,
    [PROTO_GPIO] = nrf51_gpio_udp_handler
};

void * packet_handler_context[PROTO_TOTAL] = {0x0};

//TODO: Use global prefix g_?
static nrf51_udp_conn udp_conn;

static DeviceState * nvic;
static nrf51_boot_info bootinfo;
static ppi_global_state ppis;

/*
 * Almost all modules are implemented
 * in a way that they can be duplicated
 * by only defining an unused memory
 * IO region.
 * For GPIO and GPIOTE modules, the
 * only exception is that they use
 * global variables to share state
 * information.
 *
 * Since NRF51 doesn't have any difference
 * between device models, it doesn't
 * require to be configurable from that
 * perspective but with small changes
 * NRF52 board can also be supported
 * and possibly future devices as well
 * with minimal changes. So we sill
 * have to prioritize portability.
 */

/*
 * This is a less portable solution
 * but GPIO and GPTE modules need to
 * know about each other's state.
 */
static nrf51_gpio_state * g_gpio_state;

static uint32_t seed_xsrand = 16777619; //Some prime number.

#if defined(__unix__) | defined(__APPLE__)
static void xsrand_init(void)
{
    ssize_t n;
    uint32_t seed;
    int fd = open("/dev/urandom", O_RDONLY);
    if(fd < 0){
        printf("cannon init random seed\n");
        return;
    }

    n = read(fd, &seed, sizeof(seed));

    if (n != sizeof(seed)){
        printf("unable to read required bits from urandom\n");
        return;
    }

    seed_xsrand ^= seed;

    printf("new seed: 0x%x\n", seed_xsrand);
}
#endif

/*
 * Warning: Following random function cannot
 * be used in any kind of cryptographic
 * application. It is only for educational
 * purposes.
 */
static uint32_t xsrand(void)
{
    uint32_t x = seed_xsrand;
    //Taken from https://en.wikipedia.org/wiki/Xorshift
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    seed_xsrand = x;

    return x;
}

static void nrf51_udp_send_id_handler(void * opaque, uint8_t * data, uint_fast16_t len)
{
    printf("warning: id packet received\n");
    //do nothing.
}

static void nrf51_gpio_udp_handler(void * opaque, uint8_t * data, uint_fast16_t len)
{
    nrf51_udp_gpio_hdr * gpio_msg = (nrf51_udp_gpio_hdr*)data;
    nrf51_udp_proto_hdr * hdr = (nrf51_udp_proto_hdr*)data;
    nrf51_gpio_state * s = opaque;
    uint8_t pin;
    bool state;

    if ( len != sizeof(nrf51_udp_gpio_hdr) ||
         udp_read_len(hdr) != UDP_GPIO_MSG_SZ ) {
        printf("gpio packet was corrupted, drop\n");
        return;
    }

    pin = gpio_msg->pin_state & 0x1f;
    state = gpio_msg->pin_state >> 7;

    printf("msg = set pin: %u <- %s\n", pin, state ? "HIGH" : "LOW");

   /*
    * GPIOTE module has priority on pins.
    * So if GPIOTE is configured to use
    * pins, this event must be routed
    * to GPIOTE.
    */

    if (s->owner[pin])
    {
        //Event mode = INPUT
        //Task mode = OUTPUT
        //GPIOTE.CONFIG[x] must be configured in event(input) mode.
        //Otherwise we ignore this request, because otherside
        //is configured its pin as output as well.
        if(nrf51_gpte_read_mode(s->owner[pin], s->owner_id[pin]) == GPTE_CONFIG_EVENT){
            nrf51_gpte_set_event(s->owner[pin], s->owner_id[pin], state);
        }
        else{
            printf("received input on GPIOTE task (output) pin\n");
        }

        return;
    }

    //a. Pin must be set in input mode
    //  0 == INPUT, 1 == OUTPUT
    //b. Input buffer must be connected.
    //  0 == Connected, 1 == Disconnected
    if ( !(s->REG.PINCNF[pin] & (MASK_GPIO_PINCNF_DIR | MASK_GPIO_PINCNF_INPUT)) )
    {
        //FIXME:
        //We would only set IN state here.
        //but at this stage we will only warn user.
        //We must keep the state of input (the input from the peer)
        //Rather than directly writing it into IN register.
        //This will be required when some other peripherals
        //are implemented.
    }
    else
    {
        printf("warning: pin '%u' was not configured as input "
               "and/or input buffer is disconnected\n", pin);
    }

    if (state){
        //Set
        s->REG.IN |= 1 << pin;
        printf("set pin: %u\n", pin);
    }
    else{
        //Clear
        s->REG.IN &= ~(1 << pin);
        printf("clear pin: %u\n", pin);
    }

}

static void nrf51_udp_rd_handler(void * opaque)
{
    uint8_t buffer[512];
    struct sockaddr_in si_server;
    socklen_t slen;
    nrf51_udp_conn * pCon = opaque;
    nrf51_udp_proto_hdr * hdr = (nrf51_udp_proto_hdr*)buffer;
    //si_server is not needed here...
    int len = recvfrom(pCon->fd, buffer, sizeof(buffer), 0, (struct sockaddr *) &si_server, &slen);

    if (len < 1)
    {
        printf("no data received or socket closed\n");
        return;
    }

    if (len < PROTO_HDR_SZ)
    {
        printf("packet too small, drop\n");
        return;
    }

    if (len != udp_read_len(hdr) + PROTO_HDR_SZ)
    {
        puts("drop, packet incomplete");
        return;
    }

    udp_conn.uBytesRcvd += (unsigned int) len;

    if (hdr->reserved != 0x00)
    {
        printf("reserved byte is non-zero, drop\n");
        return;
    }

    printf("received data, proto_type: %u\n", hdr->proto_type);

    if (hdr->proto_type < PROTO_TOTAL)
    {
        packet_handlers[hdr->proto_type](
            packet_handler_context[hdr->proto_type],
            buffer, (uint_fast16_t) len);
    }
    else
    {
        printf("packet type unknown, drop\n");
    }
}

static void nrf51_udp_wr_handler(void *opaque)
{
    udp_conn.bCanWrite = true;
    qemu_set_fd_handler(udp_conn.fd, nrf51_udp_rd_handler, NULL, opaque);
}

int nrf51_udp_send(void * data, size_t len)
{
    int sent;

    if (udp_conn.fd < 0)
    {
        printf("socket not initialized\n");
        return -1;
    }

    if (!udp_conn.bCanWrite)
    {
        //TODO: we actually 'might be' able to write
        //before event loop checks for fds.
        //It would be good to check if socket is writeable here instead.
        return -1;
    }

    udp_conn.bCanWrite = false;

    printf("sending data, len: %lu\n", len);
    sent = sendto(udp_conn.fd, data, len, 0,
            (struct sockaddr *) &udp_conn.si_server, sizeof(udp_conn.si_server));

    qemu_set_fd_handler(udp_conn.fd, nrf51_udp_rd_handler, nrf51_udp_wr_handler, &udp_conn);

    if (sent < 0)
    {
        printf("unable to send udp packet\n");
        //TODO: keep track of dropped data, if required.
        return -1;
    }
    else if(sent != len)
    {
        printf("not all bytes were sent\n");
    }

    udp_conn.uBytesSent += (uint_fast32_t) sent;
    printf("total bytes out: %u\n", udp_conn.uBytesSent);
    return sent;

}

void nrf51_udp_fill_hdr(nrf51_udp_proto_hdr *hdr, uint8_t proto_type, uint16_t len)
{
    hdr->proto_type = proto_type;
    hdr->reserved = 0x0;
    udp_set_len(hdr, len);
}

static void nrf51_udp_init(void)
{
    udp_conn = (nrf51_udp_conn){
        .fd = -1,
        .bCanWrite = true,
        .uBytesRcvd = 0,
        .uBytesSent = 0
    };

    udp_conn.fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if ( udp_conn.fd == -1 )
    {
        printf("unable to create UDP socket");
    }
    else
    {
        memset((char *) &udp_conn.si_server, 0x0, sizeof(udp_conn.si_server));
        udp_conn.si_server.sin_family = AF_INET;
        udp_conn.si_server.sin_port = htons(SERVER_PORT);
        
        if (inet_aton(SERVER_ADDR, &udp_conn.si_server.sin_addr) == 0)
        {
            printf("inet_aton() failed\n");
            close(udp_conn.fd);
            udp_conn.fd = -1;
        }
    }

    if(udp_conn.fd > -1)
    {
        nrf51_udp_send_id_hdr pkt;

        qemu_set_fd_handler(udp_conn.fd, nrf51_udp_rd_handler, NULL, &udp_conn);

        nrf51_udp_fill_hdr((nrf51_udp_proto_hdr*)&pkt,
                            PROTO_SEND_ID, UDP_SEND_ID_MSG_SZ);
        udp_set_uint16(pkt.id, nrf_id);
        nrf51_udp_send(&pkt, sizeof(pkt));
    }
}

static void nrf51_init(MachineState *ms)
{
    int k;

    bootinfo.kernel_filename = ms->kernel_filename;

    printf("[DP] Kernel file: %s\n", ms->kernel_filename);

    if (nrf_id >= 0xFFFF)
    {
        printf("Please specify device id with -nrf-id <id>\n");
        exit(1);
    }
#if defined(__unix__) || defined(__APPLE__)
    xsrand_init();
#else
#warning "No random seed initializer function is defined for this platform."
#endif

    MemoryRegion *system_memory = get_system_memory();

    /* SRAM */
    memory_region_init_ram(&bootinfo.sram, NULL, "nrf51_sram", SRAM_32K,
                           &error_fatal);
    memory_region_add_subregion(system_memory, NRF51_SRAM_BASE, &bootinfo.sram);

    //TODO: null check, can this even fail?
    nvic = armv7m_init(system_memory, NRF51_FLASH_SIZE_IN_BYTES, NUM_IRQ_LINES,
                       ms->kernel_filename, ms->cpu_type);

    //Create global udp connection for multi instance communication.
    nrf51_udp_init();

    //Create GPIO Module
    sysbus_create_simple(TYPE_NRF51_GPIO, GPIO_BASE, NULL);

    //Create GPTE Module
    sysbus_create_simple(TYPE_NRF51_GPTE, GPTE_BASE, NULL);

    //Create CLOCK Module
    sysbus_create_simple(TYPE_NRF51_CLOCK, POWER_CLOCK_BASE, NULL);

    //Create RTC Modules
    for (k = 0; k < RTC_TOTAL; k++)
    {
        sysbus_create_simple(TYPE_NRF51_RTC, RTC_BASE_IRQ_LIST[k].base, NULL);
    }

    //Create TIMER Modules
    for (k = 0; k < TIMER_TOTAL; k++)
    {
        sysbus_create_simple(TYPE_NRF51_TIMER, TIMER_CONFIG[k].base, NULL);
    }

    //Create ADC Module
    sysbus_create_simple(TYPE_NRF51_ADC, ADC_BASE, NULL);

    //Create UART Module
    sysbus_create_simple(TYPE_NRF51_UART, UART_BASE, NULL);

    /* FIXME:
     * The ECB, CCM, and AAR share the same AES module.
     * The ECB will always have lowest priority and if
     * there is a sharing conflict during encryption,
     * the ECB operation will be aborted and an ERRORECB
     * event will be generated.
     */
    //Create AES Module
    sysbus_create_simple(TYPE_NRF51_ECB, ECB_BASE, NULL);

    //Create RADIO Module
    sysbus_create_simple(TYPE_NRF51_RADIO, RADIO_BASE, NULL);

    //Create RNG Module
    sysbus_create_simple(TYPE_NRF51_RNG, RNG_BASE, NULL);

    //Create NVM Module
    sysbus_create_simple(TYPE_NRF51_NVM, NVM_BASE, NULL);

    //Create FICR Module
    sysbus_create_simple(TYPE_NRF51_FICR, FICR_BASE, NULL);

    //Create UICR Module
    sysbus_create_simple(TYPE_NRF51_UICR, UICR_BASE, NULL);

    //Create WDT Module
    sysbus_create_simple(TYPE_NRF51_WDT, WDT_BASE, NULL);

    //Create PPI Module
    sysbus_create_simple(TYPE_NRF51_PPI, PPI_BASE, NULL);

    //Create CCM Module
    sysbus_create_simple(TYPE_NRF51_CCM, CCM_BASE, NULL);

    //Used in is_manual_peripheral_setup_needed, called by SystemInit
    create_unimplemented_device("UNKNOWN", 0xF0000000, 0x1000);

}

static void nrf51_machine_init(MachineClass *mc)
{
    mc->desc = "nRF51 Series Development Board";
    mc->init = nrf51_init;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("cortex-m3");
}

#define UART_STARTRX_TASK (1<<0)
#define UART_STARTTX_TASK (1<<2)

static uint64_t nrf51_nvm_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_nvm_state *s = opaque;

    switch(offset)
    {
        case O_NVM_READY:
        {
            return 1;
        }

        case O_NVM_CONFIG:
            return s->REG.CONFIG;
    }
    return (uint64_t)-1;
}

static void nrf51_nvm_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_nvm_state *s = opaque;
    switch(offset)
    {
        case O_NVM_CONFIG:
            s->REG.CONFIG = value;
            break;

            break;

        case O_NVM_ERASEPCR0: //Code Region 0
        case O_NVM_ERASEPCR1: //Code Region 1
            nrf51_nvm_erase_page(s, value, offset == O_NVM_ERASEPCR0);
            break;
    }
}

static void nrf51_nvm_erase_page(nrf51_nvm_state * s, uint32_t offset, bool is_region_0)
{
    const uint8_t access_mode = s->REG.CONFIG & 0x3;

    if (access_mode != NVM_CONFIG_EEN)
    {
        return;
    }

    if ( (offset % NRF51_CODEPAGESIZE) || (offset >= NRF51_FLASH_SIZE_IN_BYTES) )
    {
        fprintf(stderr, "warning: erase page, incorrect ptr: 0x%x\n", offset);
        return;
    }

    uint8_t * const ram_ptr = memory_region_get_ram_ptr(&s->flash);

    memset(ram_ptr + offset, 0xFF, NRF51_CODEPAGESIZE);
    nrf51_nvm_set_dirty_bit(s, offset / NRF51_CODEPAGESIZE);
}

static bool nrf51_nvm_fopen_kernel(nrf51_nvm_state * s)
{
    if (!s->fpkernel)
    {
        s->fpkernel = fopen(bootinfo.kernel_filename, "r+b");
    }

    const bool ret = !!s->fpkernel;
    if (!ret)
    {
        fprintf(stderr, "unable to open kernel file\n");
    }
    return ret;
}

static void nrf51_nvm_fclose_kernel(nrf51_nvm_state * s)
{
    if (s->fpkernel)
    {
        fflush(s->fpkernel);
        fclose(s->fpkernel);
        s->fpkernel = NULL;
    }
}

static void nrf51_nvm_sync_to_file(void * opaque)
{
    nrf51_nvm_state * s = opaque;

    uint8_t * const ram_ptr = memory_region_get_ram_ptr(&s->flash);

    if (!nrf51_nvm_fopen_kernel(s))
    {
        //TODO: exit or warning?
        exit(1);
    }

    for (int i = 0; i < sizeof(s->nvm_dirty_bits); i++)
    {
        const uint8_t bits = s->nvm_dirty_bits[i];
        if (bits)
        {
            for (int p = 0; p < 8; p++)
            {
                if ((bits >> p) & 1)
                {
                    bool ok;
                    const uint8_t page = i * 8 + p;
                    const long offset = page * NRF51_CODEPAGESIZE;
                    //TODO: remove debug output
                    printf("sync dirty page: %u\n", page);
                    ok = !fseek(s->fpkernel, offset, SEEK_SET);
                    ok = ok && (fwrite(ram_ptr + offset, 1,
                                       NRF51_CODEPAGESIZE, s->fpkernel) == NRF51_CODEPAGESIZE);

                    if (!ok)
                    {
                        //TODO: error handling.
                        fprintf(stderr, "file I/O error\n");
                        exit(1);
                    }
                }
            }
            s->nvm_dirty_bits[i] = 0;
        }
    }

    nrf51_nvm_fclose_kernel(s);
}

static void nrf51_nvm_set_dirty_bit(nrf51_nvm_state * s, uint_fast32_t page_num)
{
    s->nvm_dirty_bits[page_num/8] |= 1 << (page_num % 8);

    if (!s->file_sync)
    {
        QEMUBH *bh = qemu_bh_new(nrf51_nvm_sync_to_file, s);

        s->file_sync = ptimer_init(bh, PTIMER_POLICY_DEFAULT);
        if (!s->file_sync)
        {
            fprintf(stderr, "ptimer_init errror!");
            exit(1);
        }
    }

    //Delay file system writes for a second
    ptimer_stop(s->file_sync);
    ptimer_set_freq(s->file_sync, 1);
    ptimer_set_count(s->file_sync, 1);
    ptimer_run(s->file_sync, 1);
}

static void nrf51_nvm_data_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_nvm_state * const s = opaque;
    const uint8_t access_mode = s->REG.CONFIG & 0x3;
    const uint_fast32_t page_num = offset / NRF51_CODEPAGESIZE; //Page number;
    if (access_mode != NVM_CONFIG_WEN)
    {
        if (access_mode > NVM_CONFIG_EEN)
        {
            fprintf(stderr, "warning: incorrect NVM access mode configuration\n");
        }
        return;
    }

    uint8_t * const ram_ptr = memory_region_get_ram_ptr(&s->flash);

    //FIXME: check that offset is aligned. Is it required???

    printf("NVM write at: 0x%x = 0x%x\n", (uint32_t) offset, (uint32_t)value);

    uint32_t * const ptr = (uint32_t*) (ram_ptr + offset);

    *ptr &= value; //NVM behaves in that way if area is not erased.

    nrf51_nvm_set_dirty_bit(s, page_num);
}

static uint64_t nrf51_uicr_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_uicr_state *s = opaque;

    switch(offset)
    {
        //Readback protection is not implemented for QEMU
        case O_UICR_RBPCONF: return (uint64_t)-1;

        case O_UICR_CLENR0:         return s->REG.CLENR0;
        case O_UICR_XTALFREQ:       return s->REG.XTALFREQ;
        case O_UICR_BOOTLOADERADDR: return s->REG.BOOTLOADERADDR;

        //FIXME: Not supported??
        case O_UICR_FWID:
        //Reserved for Nordic FW/HW design
        case O_UICR_NRFFW1 ... O_UICR_NRFFW14:
        case O_UICR_NRFHW1 ... O_UICR_NRFHW11:
            return (uint64_t)-1;
    }

    fprintf(stderr, "UICR unknown reg read: 0x%llx\n", offset);
    return (uint64_t)-1;
}

static void nrf51_uicr_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_uicr_state *s = opaque;
    bool save_regs = false;

    switch(offset)
    {
        //FIXME: mass erase
        case O_UICR_CLENR0:
            if (s->REG.CLENR0 == (uint32_t) - 1)
            {
                s->REG.CLENR0 = value;
                save_regs = true;
            }
            break;

        case O_UICR_XTALFREQ:
        {
            const uint32_t xtf = (value & 0xFF);
            if (s->REG.XTALFREQ != xtf)
            {
                s->REG.XTALFREQ = xtf;
                save_regs = true;
            }
            break;
        }

        case O_UICR_FWID:

        //FIXME: Missing info in docs.
        case O_UICR_BOOTLOADERADDR:
            if (s->REG.BOOTLOADERADDR != value)
            {
                s->REG.BOOTLOADERADDR = value;
                save_regs = true;
            }
            break;

        case O_UICR_CUSTOMER0 ... O_UICR_CUSTOMER31:
        {
            const int idx = (offset - O_UICR_CUSTOMER0) / 4;
            if (s->REG.CUSTOMER[idx] != value)
            {
                s->REG.CUSTOMER[idx] = value;
                save_regs = true;
            }
        }
        //Reserved for Nordic FW/HW design
        case O_UICR_NRFFW1 ... O_UICR_NRFFW14:
        case O_UICR_NRFHW1 ... O_UICR_NRFHW11:
            break;

        default:
            fprintf(stderr, "UICR unknown reg write: 0x%llx\n", offset);
            break;
    }

    if (save_regs)
    {
        nrf51_uicr_save_regs(s);
    }
}

static uint64_t nrf51_gpio_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    nrf51_gpio_state * s = opaque;
    switch (offset)
    {
        //Following 3 registers return the same value on read.
        case O_GPIO_OUT:
        case O_GPIO_OUTSET:
        case O_GPIO_OUTCLR:
            return s->REG.OUT; //Reading OUTSET/OUTCLR returns value of OUT.

        case O_GPIO_IN:
            return s->REG.IN;

        //Similar to OUT/OUTSET/OUTCLR register.
        case O_GPIO_DIR:
        case O_GPIO_DIRSET:
        case O_GPIO_DIRCLR:
            return s->REG.DIR;

        case O_GPIO_PIN_CNF0 ... O_GPIO_PIN_CNF31:
        {
            //Calculate index for the pin configuration registers.
            const unsigned int idx = (offset - O_GPIO_PIN_CNF0) / 4;
            return s->REG.PINCNF[idx];
        }
        default:
            return 0xFFFFFFFF;
    }

    printf("[uart] not implemented: 0x%u\n",
        (unsigned int) offset);

    return 0xFFFFFFFF;
}

static void nrf51_gpio_send_single(uint8_t pin, uint8_t state)
{
    nrf51_udp_gpio_hdr pkt;
    nrf51_udp_fill_hdr(&pkt.proto_hdr, PROTO_GPIO, 1);
    pkt.pin_state = ((!!state) << 7) | (pin & 0x1f);
    nrf51_udp_send(&pkt, sizeof(pkt));
}

static void nrf51_gpte_task_out(nrf51_gpte_state *s, int idx)
{
    uint8_t state;
    //TODO: Check if PSEL can be changed after CONFIG.
    int pin = nrf51_gpte_read_psel(s, idx);
    int polarity = nrf51_gpte_read_polarity(s, idx);

    switch(polarity)
    {
        case GPTE_CONFIG_NONE:
            //Don't sync
            return;

        case GPTE_CONFIG_RISING:
            state = 1;
            break;

        case GPTE_CONFIG_FALLING:
            state = 0;
            break;

        case GPTE_CONFIG_TOGGLE:
            state = !s->io_state[idx];
            break;
    }

    if (s->io_state[idx] != state){
        //Sync only if state is changed.
        s->io_state[idx] = state;
        nrf51_gpio_send_single(pin, state);
    }
}

static void nrf51_gpio_sync_state(nrf51_gpio_state *s, uint32_t old_value)
{
    int i;
    uint32_t modified = s->REG.OUT ^ old_value;
    //Send modified pin states.
    for (i = 0; i < 32; i++, modified >>= 1)
    {
        if (modified & 0x1)
        {
            if (s->owner[i]){
                printf("pin %d owned by GPIOTE, don't sync\n", i);
                //If this pin is controled by GPIOTE module
                //we must skip it when state change was attempted
                //from GPIO module.
                continue;
            }
            //TODO: Check if REG.PINCNF affects REG.DIR

            //Check that it is output pin.
            if (/*GET_BIT(s->REG.DIR, i) && */ GET_BIT(s->REG.PINCNF[i], BIT_GPIO_PINCNF_DIR))
            {
                const uint8_t state = (s->REG.OUT >> i) & 0x1;
                printf("send single: %0x\n", state);
                nrf51_gpio_send_single(i, state);
            }
            else
            {
                printf("won't sync input pin: %d\n", i);
            }
        }
    }
}

static void nrf51_gpio_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    nrf51_gpio_state * s = opaque;
    uint32_t old_value;

    switch (offset)
    {
        case O_GPIO_OUT:
            if (s->REG.OUT != value)
            {
                old_value = s->REG.OUT;
                s->REG.OUT = value;
                nrf51_gpio_sync_state(s, old_value);
            }
            break;

        /*
         * Set individual bits of OUT register.
         * Writing 0 has no effect
         */
        case O_GPIO_OUTSET:
            old_value = s->REG.OUT;
            s->REG.OUT |= (uint32_t) value;
            if (old_value != s->REG.OUT)
            {
                nrf51_gpio_sync_state(s, old_value);
            }
            break;
        /*
         * Clear individual bits of OUT register.
         * Writing 0 has no effect.
         * Writing 1 clears bit.
         */
        case O_GPIO_OUTCLR:
            old_value = s->REG.OUT;
            s->REG.OUT &= (uint32_t) ~value;
            if (old_value != s->REG.OUT)
            {
                nrf51_gpio_sync_state(s, old_value);
            }
            break;

        case O_GPIO_DIR:
            printf("Set DIR: 0x%llx\n", value);
            s->REG.DIR = (uint32_t) value;
            break;

        case O_GPIO_DIRSET:
            s->REG.DIR |= (uint32_t) value;
            break;

        case O_GPIO_DIRCLR:
            s->REG.DIR &= (uint32_t) ~value;
            break;

        case O_GPIO_PIN_CNF0 ... O_GPIO_PIN_CNF31:
        {
            //Calculate index for the pin configuration registers.
            const unsigned int idx = (offset - O_GPIO_PIN_CNF0) / 4;
            s->REG.PINCNF[idx] = value;
            break;
        }
    }
}

static uint64_t nrf51_gpte_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_gpte_state *s = opaque;

    switch(offset)
    {
        //Events
        case O_GPTE_IN0: return s->REG.IN[0];
        case O_GPTE_IN1: return s->REG.IN[1];
        case O_GPTE_IN2: return s->REG.IN[2];
        case O_GPTE_IN3: return s->REG.IN[3];
        case O_GPTE_PORT: return s->REG.PORT;

        //Registers
        case O_GPTE_INTEN:
        case O_GPTE_INTENSET:
        case O_GPTE_INTENCLR:
            return s->REG.INTEN;

        case O_GPTE_CONFIG0: return s->REG.CONFIG[0];
        case O_GPTE_CONFIG1: return s->REG.CONFIG[1];
        case O_GPTE_CONFIG2: return s->REG.CONFIG[2];
        case O_GPTE_CONFIG3: return s->REG.CONFIG[3];

    }

    printf("gpiote rd unknown register\n");
    return (uint64_t)-1;
}

static void nrf51_gpte_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_gpte_state *s = opaque;

    switch(offset)
    {
        //Tasks
        case O_GPTE_OUT0 ... O_GPTE_OUT3:
        {
            //
            //FIXME: Check if PSEL can be changed after CONFIG.
            //

            int idx = (offset - O_GPTE_OUT0) / 4;
            if(nrf51_gpte_read_mode(s, idx) == GPTE_CONFIG_TASK){
                nrf51_gpte_task_out(s, idx);
            }
            break;
        }

        //Events
        case O_GPTE_IN0:
            s->REG.IN[0] = value;
            break;

        case O_GPTE_IN1:
            s->REG.IN[1] = value;
            break;

        case O_GPTE_IN2:
            s->REG.IN[2] = value;
            break;

        case O_GPTE_IN3:
            s->REG.IN[3] = value;
            break;

        case O_GPTE_PORT:
            s->REG.PORT = value;
            break;

        //Registers
        case O_GPTE_INTEN:
            s->REG.INTEN = value;
            break;

        case O_GPTE_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_GPTE_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_GPTE_CONFIG0 ... O_GPTE_CONFIG3:
        {
            //
            //FIXME: Check if PSEL can be changed after CONFIG.
            //

            int mode;
            int psel;
            int idx = (offset - O_GPTE_CONFIG0) / 4;
            s->REG.CONFIG[idx] = value;

            mode = nrf51_gpte_read_mode(s, idx);
            psel = nrf51_gpte_read_psel(s, idx);
            if (mode == GPTE_CONFIG_EVENT || mode == GPTE_CONFIG_TASK){
                //Assign an owner for this pin
                g_gpio_state->owner[psel] = s;
                g_gpio_state->owner_id[psel] = idx;

                if (mode == GPTE_CONFIG_TASK){
                    s->io_state[idx] = GET_BIT(s->REG.CONFIG[idx], BIT_GPTE_CONFIG_OUTINIT);
                    nrf51_gpio_send_single(psel, s->io_state[idx]);
                }
                else{
                    s->io_state[idx] = 0;
                }
            }
            break;
        }

        default:
            printf("gpte wr unknown register\n");
            break;
    }
}

static uint64_t nrf51_clock_read(void *opaque, hwaddr offset,
                                 unsigned int size)
{
    nrf51_clock_state *s = opaque;

    switch(offset)
    {
        //Events
        case O_CLOCK_HFCLKSTARTED: return s->REG.HFCLKSTARTED;
        case O_CLOCK_LFCLKSTARTED: return s->REG.LFCLKSTARTED;
        case O_CLOCK_DONE: return s->REG.DONE;
        case O_CLOCK_CTTO: return s->REG.CTTO;

        //Registers
        case O_CLOCK_INTENSET:
        case O_CLOCK_INTENCLR:
            return s->REG.INTEN;

        case O_CLOCK_HFCLKRUN: return s->REG.HFCLKRUN;
        case O_CLOCK_HFCLKSTAT: return s->REG.HFCLKSTAT;
        case O_CLOCK_LFCLKRUN: return s->REG.LFCLKRUN;
        case O_CLOCK_LFCLKSTAT: return s->REG.LFCLKSTAT;
        case O_CLOCK_LFCLKSRCCOPY: return s->REG.LFCLKSRCCOPY;
        case O_CLOCK_LFCLKSRC: return s->REG.LFCLKSRC;
        case O_CLOCK_CTIV: return s->REG.CTIV;
        case O_CLOCK_XTALFREQ: return s->REG.XTALFREQ;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "CLOCK: unk read at 0x%x\n", (uint32_t)offset);

    return (uint64_t)-1;
}

static void nrf51_clock_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned int size)
{
    nrf51_clock_state *s = opaque;

    switch(offset)
    {
        //Tasks
        case O_CLOCK_HFCLKSTART:
            if (value){
                s->REG.HFCLKSTARTED = 1;
                s->REG.HFCLKRUN = 1;
                //Set running state
                s->REG.HFCLKSTAT = 1<<16;
                if (GET_BIT(s->REG.INTEN, BIT_CLOCK_INT_HFCLKSTARTED)){
                    qemu_irq_pulse(s->irq);
                }
            }
            break;

        case O_CLOCK_HFCLKSTOP:
            if(value){
                s->REG.HFCLKSTARTED = 0;
                s->REG.HFCLKRUN = 0;
                s->REG.HFCLKSTAT = 0;
            }
            break;

        case O_CLOCK_LFCLKSTART:
            if(value){
                s->REG.LFCLKSRCCOPY = s->REG.LFCLKSRC;
                s->REG.LFCLKSTARTED = 1;
                s->REG.LFCLKRUN = 1;
                //Set running state
                s->REG.LFCLKSTAT = CLOCK_STAT_RUNNING | (s->REG.LFCLKSRC & MASK_CLOCK_LFCLKSRC);
                if (GET_BIT(s->REG.INTEN, BIT_CLOCK_INT_LFCLKSTARTED)){
                    qemu_irq_pulse(s->irq);
                }
            }
            break;

        case O_CLOCK_LFCLKSTOP:
            if(value){
                s->REG.LFCLKSTARTED = 0;
                s->REG.LFCLKRUN = 0;
                s->REG.LFCLKSTAT = 0;
            }
            break;

        case O_CLOCK_CAL:
            if(value){
                s->REG.DONE = 1;
                if(GET_BIT(s->REG.INTEN, BIT_CLOCK_INT_DONE)){
                    qemu_irq_pulse(s->irq);
                }
            }
            break;

        //These tasks are HW level (calibration)
        case O_CLOCK_CTSTART:
        case O_CLOCK_CTSTOP:
            break;

        //Events
        case O_CLOCK_HFCLKSTARTED:
            s->REG.HFCLKSTARTED = value;
            break;
        case O_CLOCK_LFCLKSTARTED:
            s->REG.LFCLKSTARTED = value;
            break;
        case O_CLOCK_DONE:
            s->REG.DONE = value;
            break;
        case O_CLOCK_CTTO:
            s->REG.CTTO = value;
            break;

        //Registers
        case O_CLOCK_INTENSET:
            s->REG.INTEN |= value;
            break;
        case O_CLOCK_INTENCLR:
            s->REG.INTEN &= ~value;
            break;
        case O_CLOCK_LFCLKSRC:
            s->REG.LFCLKSRC = value & MASK_CLOCK_LFCLKSRC;
            break;
        case O_CLOCK_CTIV:
            s->REG.CTIV = value & 0x7F;
            break;
        case O_CLOCK_XTALFREQ:
            s->REG.XTALFREQ = value & 0xFF;
            break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR, "CLOCK: unk write at 0x%x\n", (uint32_t)offset);
            break;
    }

}

static int64_t get_clock_virt(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
}

static int64_t get_ns_per_tick(int64_t freq)
{
    static const uint64_t CLOCK_SCALE = 1; //10
    return NANOSECONDS_PER_SECOND * CLOCK_SCALE / freq;
}

static double get_ns_per_tick_d(int64_t freq)
{
    static const double CLOCK_SCALE = 1.0;
    return ((double)NANOSECONDS_PER_SECOND) * CLOCK_SCALE / ((double)freq);
}

static uint64_t nrf51_rtc_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    nrf51_rtc_state * s = opaque;
    switch(offset)
    {
        /* Tasks */
        case O_RTC_START:
        case O_RTC_STOP:
        case O_RTC_CLEAR:
        case O_RTC_TRIGOVRFLW:
            return (uint64_t) -1;

        /* Events */
        case O_RTC_TICK: return s->REG.TICK;
        case O_RTC_OVRFLW: return s->REG.OVRFLW;
        case O_RTC_COMPARE0: return s->REG.COMPARE[0];
        case O_RTC_COMPARE1: return s->REG.COMPARE[1];
        case O_RTC_COMPARE2: return s->REG.COMPARE[2];
        case O_RTC_COMPARE3: return s->REG.COMPARE[3];

        /* Registers */
        case O_RTC_COUNTER:
        {
            uint32_t ret;
            if (s->bRunning)
            {
                const int64_t ns_passed = get_clock_virt() - s->rtc_begin_ns;
                const int64_t ticks = (int64_t)((double)ns_passed / s->ns_per_tick);
                ret = s->REG.COUNTER = ((uint32_t)ticks) & MASK_RTC_COUNTER;
            }
            else
            {
                ret = s->REG.COUNTER;
            }
            return ret;
        }
        case O_RTC_PRESCALER: return s->REG.PRESCALER;
        case O_RTC_CC0: return s->REG.CC[0];
        case O_RTC_CC1: return s->REG.CC[1];
        case O_RTC_CC2: return s->REG.CC[2];
        case O_RTC_CC3: return s->REG.CC[3];

        case O_RTC_INTEN:
        case O_RTC_INTENSET:
        case O_RTC_INTENCLR:
            return s->REG.INTEN;

        case O_RTC_EVTEN:
        case O_RTC_EVTENSET:
        case O_RTC_EVTENCLR:
            return s->REG.EVTEN;

    }

    return 0xffffffff;
}

static void nrf51_rtc_set_next(nrf51_rtc_state * s)
{
    g_assert(s->REG.COUNTER < MASK_RTC_COUNTER); //unlikely
    uint32_t dist = (MASK_RTC_COUNTER - s->REG.COUNTER + 1); //Closest event (overflow)
    s->next_tick = MASK_RTC_COUNTER + 1; //zero on overflow
    for (int i = 0; i < RTC_NUM_CC_REGS; i++)
    {
        const uint32_t cc_dist = (s->REG.CC[i] > s->REG.COUNTER) ?
            (s->REG.CC[i] - s->REG.COUNTER) :
            (MASK_RTC_COUNTER - s->REG.CC[i] + s->REG.COUNTER);
            //printf("[RTC%d] compare 0x%x vs 0x%x\n",s->num_instance, dist, cc_dist);
            if (cc_dist < dist)
            {
                dist = cc_dist;
                s->next_tick  = s->REG.CC[i];
            }
    }

    double future = s->ns_per_tick * (double)dist;
    const int64_t next = (int64_t) future + get_clock_virt();//s->rtc_begin_ns;
    timer_mod_ns(s->qtimer, next);
}

static void nrf51_rtc_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
        nrf51_rtc_state * s = opaque;

        qemu_mutex_lock(&s->mtx);
        switch(offset)
        {
            /* Tasks */
            case O_RTC_START:
                g_assert(!s->bRunning); //guest error
                if (!value || s->bRunning) //TODO: Compare '1' or any value?
                    break;

                //TODO: Frequency can be different depending on the clock source
                //Start task for counter.
                if (s->bRunning)
                {
                    printf("[RTC] WARNING tried to start already running RTC\n");
                }

                const int64_t guest_freq = 32768 / (s->REG.PRESCALER + 1);
                s->ns_per_tick = get_ns_per_tick_d(guest_freq);
                s->rtc_begin_ns = get_clock_virt();
                s->bRunning = true;
                nrf51_rtc_set_next(s);

                printf("[RTC] started\n");
                //TODO: Do we even need to save start?
                break;

            case O_RTC_STOP:
                if (!value)
                    break;
                printf("[RTC] stopped\n");
                s->bRunning = false;
                break;

            case O_RTC_CLEAR:
                printf("[RTC] Clear\n");
                s->REG.COUNTER = 0;
                if (s->bRunning)
                {
                    s->rtc_begin_ns = get_clock_virt();
                    nrf51_rtc_set_next(s);
                }
                break;

            case O_RTC_TRIGOVRFLW:
                s->REG.COUNTER = 0xFFFFF0;
                break;

            /* Events */
            case O_RTC_TICK:
                s->REG.TICK = value;
                break;

            case O_RTC_OVRFLW:
                s->REG.OVRFLW = value;
                break;

            case O_RTC_COMPARE0:
                s->REG.COMPARE[0] = value;
                break;

            case O_RTC_COMPARE1:
                s->REG.COMPARE[1] = value;
                break;
            case O_RTC_COMPARE2:
                s->REG.COMPARE[2] = value;
                break;
            case O_RTC_COMPARE3:
                s->REG.COMPARE[3] = value;
                break;

            /* Registers */
            case O_RTC_PRESCALER:
                if (s->bRunning) //Read only when running.
                    break;
                //Prescaler size is 12-bit
                s->REG.PRESCALER = value & ((1<<12)-1);
                printf("Prescaler set to: %u\n", s->REG.PRESCALER);
                break;

            case O_RTC_INTEN:
                s->REG.INTEN = value;
                break;

            case O_RTC_INTENSET:
                s->REG.INTEN |= value;
                break;

            case O_RTC_INTENCLR:
                s->REG.INTEN &= ~value;
                break;

            case O_RTC_EVTEN:
                s->REG.EVTEN = value;
                break;

            case O_RTC_EVTENSET:
                s->REG.EVTEN |= value;
                break;

            case O_RTC_EVTENCLR:
                s->REG.EVTEN &= ~value;
                break;

            case O_RTC_CC0 ... O_RTC_CC3:
                {
                    int idx = (offset - O_RTC_CC0) / 4;
                    s->REG.CC[idx] = value & MASK_RTC_COUNTER;
                    s->REG.COUNTER = nrf51_rtc_read(s, O_RTC_COUNTER, 4); //TODO: when mutex used, DEADLOCK
                    if (s->bRunning) nrf51_rtc_set_next(s);
                }
                break;

            default:
                printf("[rtc] unk wr request at 0x%x, value: %llu\n", (unsigned int) offset, (unsigned long long)value);
                break;
        }

        qemu_mutex_unlock(&s->mtx);
}


static void nrf51_rtc_timer(void * opaque)
{
    nrf51_rtc_state  * const s = opaque;
    qemu_mutex_lock(&s->mtx); //TODO: use mutex for read as well.
    if (!s->bRunning)
    {
        qemu_mutex_unlock(&s->mtx);
        return;
    }
    s->REG.COUNTER = s->next_tick;
    nrf51_rtc_tick(s);
    const int64_t ns_passed = (int64_t) (((double)s->REG.COUNTER) * s->ns_per_tick);
    s->rtc_begin_ns = get_clock_virt() - ns_passed;
    nrf51_rtc_set_next(s);
    qemu_mutex_unlock(&s->mtx);
}

static void nrf51_rtc_tick(void *opaque)
{
    int i;
    uint32_t cc_bit;
    bool pulse = false;
    nrf51_rtc_state * s = opaque;

    /*
     * RTC has slightly different event/task system.
     * Ref. Man. p.105 Figure 34.
     * An event is set if the event is enabled (EVTEN) or the interrupt (INTEN).
     */
    if (s->REG.COUNTER > MASK_RTC_COUNTER)
    {
        s->REG.COUNTER = 0;

        if (s->REG.EVTEN & MASK_RTC_INTEN_OVERFLW)
        {
            s->REG.OVRFLW = 1;
            ppi_event_filter(s->base + O_RTC_OVRFLW);
        }

        if (s->REG.INTEN & MASK_RTC_INTEN_OVERFLW)
        {
            s->REG.OVRFLW = 1;
            pulse = true;
            puts("rtc irq overflw");
        }
    }

    if (s->REG.EVTEN & MASK_RTC_INTEN_TICK)
    {
        s->REG.TICK = 1;
        ppi_event_filter(s->base + O_RTC_TICK);
    }

    if (s->REG.INTEN & MASK_RTC_INTEN_TICK)
    {
        s->REG.TICK = 1;
        pulse = true;
        puts("rtc irq tick");
    }

    for (i = 0, cc_bit = MASK_RTC_INTEN_COMPARE0; i < RTC_NUM_CC_REGS; i++)
    {
        if (s->REG.CC[i] == s->REG.COUNTER) //Does COUNTER match CC?
        {
            if (s->REG.EVTEN & cc_bit)
            {
                s->REG.COMPARE[i] = 1;
                ppi_event_filter(s->base + O_RTC_COMPARE0 + i * 4);
            }

            if (s->REG.INTEN & cc_bit) //Is event enabled?
            {
                s->REG.COMPARE[i] = 1;
                pulse = true;
            }
        }
        //COMPARE0..COMPARE4 bits in INTENT register are adjacent.
        //Checking bits from 16 to 19.
        cc_bit <<= 1;
    }

    if (pulse)
        qemu_irq_pulse(s->irq);

}

static uint64_t nrf51_adc_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    //printf("[adc] rd at offset: 0x%x\n", (unsigned int) offset);
    nrf51_adc_state * s = opaque;

    switch(offset)
    {
        case O_ADC_CONFIG:
            return s->REG.CONFIG;

        case O_ADC_RESULT:
            return s->REG.RESULT;

        case O_ADC_ENABLE:
            return s->REG.ENABLE;

        case O_ADC_BUSY:
            return !!s->bConversionActive;

        case O_ADC_INTEN:
        case O_ADC_INTENSET:
        case O_ADC_INTENCLR:
            return s->REG.INTEN;

        case O_ADC_END:
            return s->REG.END;

    }
    return 0xffffffff;
}

static void nrf51_adc_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    nrf51_adc_state * s = opaque;

    switch(offset)
    {
        case O_ADC_START:
            //printf("ADC_START\n");
            //Check that ADC is enabled and no conversion is active.
            if (!s->REG.ENABLE || s->bConversionActive)
                break;
            s->bConversionActive = true;
            ptimer_set_freq(s->pt_conversion, 32768); //TODO: 32kHz, too low?
            ptimer_set_count(s->pt_conversion, 1);
            ptimer_run(s->pt_conversion, 1);
            break;

        case O_ADC_STOP:
            if (!s->bConversionActive)
                break;
            s->bConversionActive = false;
            ptimer_stop(s->pt_conversion);
            break;

        case O_ADC_ENABLE:
            s->REG.ENABLE = value;
            break;

        case O_ADC_CONFIG:
            s->REG.CONFIG = value;
            break;

        case O_ADC_INTEN:
            s->REG.INTEN = value;
            break;

        case O_ADC_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_ADC_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_ADC_END:
            s->REG.END = value;
            break;

        default:
            printf("[adc] unk wr request at 0x%x, value: %llu\n", (unsigned int) offset, (unsigned long long)value);
            break;
    }
}

static void nrf51_adc_conversion_complete(void *opaque)
{
    nrf51_adc_state * s = opaque;
    uint32_t uRes = 600;

    if (s->nNoiseBits > 0)
    {
        const uint32_t uMask = (1 << (s->nNoiseBits - 1)) - 1;
        s->REG.RESULT = (uRes & ~uMask) | (xsrand() & uMask);
    }
    else
    {
        s->REG.RESULT = uRes;
    }

    s->REG.END = 1;

    s->bConversionActive = false;

    if (s->REG.INTEN & 0x1) //There is only one event for ADC.
    {
        qemu_irq_pulse(s->irq);
    }

}

static void nrf51_uart_tx_task(nrf51_uart_state * const s)
{
    char chPendingByte;

    if (s->bNewByte)
    {
        /*
        printf("(%u) new tx byte: 0x%x (%1s)\n",
            ctr,
            s->REG.TXD,
            (char*)&s->REG.TXD);
        */

        chPendingByte = s->REG.TXD;
        //TODO: Would it be better to keep bytes in buffer?
        if (1 != nrf51_uart_comm_write(&chPendingByte, 1))
        {
            //Couldn't write try again later.
            return;
        }

        /*
         * Set TXDRDY event and pulse IRQ.
         */
        s->REG.TXDRDY = 0x1;
        s->bNewByte = false;
        qemu_irq_pulse(s->irq);
    }
}

static void nrf51_uart_rx_task(nrf51_uart_state * const s)
{
    //TODO: Keep stats.
    enum
    {
        BUF_SIZE = 256,
        BUF_AVA = BUF_SIZE - 1,
        BUF_LIMIT = BUF_SIZE / 4 * 3
    };

    static char pchBuf[BUF_SIZE];
    static int nHead = 0;
    static int nTail = 0;

    if (nTail <= BUF_LIMIT)
    {
        int nRead;
        //Read data from UNIX socket.
        nRead = nrf51_uart_comm_read(pchBuf, BUF_AVA - nTail);
        if (nRead > 0)
        {
            nTail += nRead;
            //Just a sanity check, can't happen.
            if (nTail > BUF_AVA || nTail < 0)
            {
                //Something really bad must have happened here.
                nTail = BUF_AVA;
                return;
            }
        }
    }

    //VM didn't handle last byte
    if (!s->bReadOk)
        return;

    if (nHead >= nTail)
    {
        //Nothing to read in buffer.
        return;
    }

    s->REG.RXD = pchBuf[nHead];
    nHead++;
    //Unset this flag so we know that data is pending to be read.
    s->bReadOk = false;
    s->REG.RXDRDY = 0x1;
    qemu_irq_pulse(s->irq);

    if (nHead == nTail)
    {
        //If we consumed all bytes, then reset cursors.
        nHead = nTail = 0;
    }
}

static void nrf51_uart_timer(void *opaque)
{
    nrf51_uart_state * const s = opaque;
    static uint32_t ctr;
    ctr++;

    if (s->uTimerTaskFlags & UART_STARTRX_TASK)
    {
        nrf51_uart_rx_task(s);
    }

    if (s->uTimerTaskFlags & UART_STARTTX_TASK)
    {
        nrf51_uart_tx_task(s);
    }

    if (!s->uTimerTaskFlags)
    {
        printf("uart timer stop\n");
        //If there is no active task, stop.
        ptimer_stop(s->ptimer);
    }
}
/*
 * TODO: Be careful with clear on read
 * register. Check if there are any others.
 */
static uint64_t nrf51_uart_read(void *opaque, hwaddr offset,
                                   unsigned int size)
{
    nrf51_uart_state * s = opaque;

    switch(offset)
    {
        //Events
        case O_UART_RXDRDY:
            return s->REG.RXDRDY;
        case O_UART_TXDRDY:
            return s->REG.TXDRDY;
        case O_UART_ERROR:
            return s->REG.ERROR;
        case O_UART_RXTO:
            return s->REG.RXTO;

        case O_UART_INTEN:
        case O_UART_INTENSET:
        case O_UART_INTENCLR:
            return s->REG.INTEN;

        //Pin select registers.
        case O_UART_PSELCTS:
            return s->REG.PSELCTS;
        case O_UART_PSELRTS:
            return s->REG.PSELRTS;
        case O_UART_PSELRXD:
            return s->REG.PSELRXD;
        case O_UART_PSELTXD:
            return s->REG.PSELTXD;

        case O_UART_RXD:
            s->bReadOk = true;
            return s->REG.RXD;
        //case O_UART_ERRORSRC:
            //TODO: Need to simulate some errors on UART.
            //break;
    }

    printf("UART rd unimplemented reg: 0x%x\n", (unsigned int)offset);
    return (uint64_t) -1;
}

static void nrf51_uart_start_timer(nrf51_uart_state * s, uint32_t uFlag)
{
    const int start = !s->uTimerTaskFlags;

    s->uTimerTaskFlags |= uFlag;

    if(start)
    {
        printf("UART timer started\n");
        //TODO: Why frequency doesn't match?
        //TODO: when freq increased, TXD is slower?
        ptimer_set_freq(s->ptimer, 8192); //TODO: 32kHz, adjust baud rate.
        ptimer_set_count(s->ptimer, 1);

        //Set limit to 1 and enable reload.
        ptimer_set_limit(s->ptimer, 1, 1);

        //Continuous mode
        ptimer_run(s->ptimer, 0);
    }

}

static void nrf51_uart_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned int size)
{
    nrf51_uart_state * s = opaque;

    switch(offset)
    {
        //Tasks
        case O_UART_STARTRX:
            if (value & 0x1)
            {
                printf("Start RX\n");
                s->bReadOk = true;
                nrf51_uart_start_timer(s, UART_STARTRX_TASK);
            }
            break;

        case O_UART_STOPRX:
            if (value){
                s->uTimerTaskFlags &= ~UART_STARTRX_TASK;
                printf("Stop RX\n");
            }
            break;

        case O_UART_STARTTX:
            if (value & 0x1)
            {
                //printf("Start TX\n");
                nrf51_uart_start_timer(s, UART_STARTTX_TASK);
            }
            break;

        case O_UART_STOPTX:
            if (value){
                s->uTimerTaskFlags &= ~UART_STARTTX_TASK;
                //printf("Stop TX\n");
            }
            break;

        case O_UART_SUSPEND:
            if(value){
                s->uTimerTaskFlags &= ~(UART_STARTTX_TASK | UART_STARTRX_TASK);
                printf("UART Suspend\n");
            }
            break;

        //Events
        case O_UART_RXDRDY:
            s->REG.RXDRDY = value;
            break;
        case O_UART_TXDRDY:
            s->REG.TXDRDY = value;
            break;
        case O_UART_ERROR:
            s->REG.ERROR = value;
            break;
        case O_UART_RXTO:
            s->REG.RXTO = value;
            break;

        case O_UART_ENABLE:
            s->REG.ENABLE = value;
            if (value == UART_ENABLE_REG_VAL)
            {
                s->bUartEnabled = true;
                printf("UART enabled.\n");
            }
            else if (value == UART_DISABLE_REG_VAL)
                s->bUartEnabled = false;
            else
            {
                //TODO: check with real hardware and see what happens.
            }
            break;

        case O_UART_INTEN:
            s->REG.INTEN = value;
            break;

        case O_UART_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_UART_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        //TODO: simulate baudrate with timers.
        case O_UART_BAUDRATE:
            s->REG.BAUDRATE = value;
            break;

        case O_UART_CONFIG:
            printf("UART CONFIG: 0x%x\n", (unsigned int) value);
            /*
             * We are not interested in parity,
             * only check HW flow control.
             */
            s->bFlowCtrlEnabled = (value & 0x1);
            s->REG.CONFIG = value;
            break;

        case O_UART_TXD:
            //printf("new tx byte\n");
            s->bNewByte = true;
            s->REG.TXD = value;
            break;
        /*
         * TODO: How to simulate disconnected (0xFFFFFFFF) state for pins?
         * Stop UART when pin is disconnected?
         */
        //Pin select registers are ignored.
        case O_UART_PSELCTS:
            s->REG.PSELCTS = value;
            break;

        case O_UART_PSELRTS:
            s->REG.PSELRTS = value;
            break;

        case O_UART_PSELRXD:
            s->REG.PSELRXD = value;
            break;

        case O_UART_PSELTXD:
            s->REG.PSELTXD = value;
            break;

        default:
            printf("UART wr unimplemented reg: 0x%x = 0x%x\n",
                (unsigned int) offset, (unsigned int) value);
            break;
    }
}

static uint64_t nrf51_rng_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_rng_state *s = opaque;

    switch(offset)
    {
        case O_RNG_VALRDY: return s->REG.VALRDY;
        case O_RNG_SHORTS: return s->REG.SHORTS;
        case O_RNG_CONFIG: return s->REG.CONFIG;
        case O_RNG_VALUE:  return s->REG.VALUE;

        case O_RNG_INTEN:
        case O_RNG_INTENSET:
        case O_RNG_INTENCLR:
            return s->REG.INTEN;

    }

    printf("[rng] unk read offset: %llx\n", offset);
    return (uint64_t)-1;
}

static void nrf51_rng_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_rng_state * s = opaque;
    switch(offset)
    {
        case O_RNG_START:
            ptimer_set_freq(s->ptimer, 32768);
            ptimer_set_count(s->ptimer, 1);
            ptimer_set_limit(s->ptimer, 1, 1);
            ptimer_run(s->ptimer, 0);
            break;

        case O_RNG_STOP:
            ptimer_stop(s->ptimer);
            break;

        case O_RNG_VALRDY:
            s->REG.VALRDY = value;
            break;

        case O_RNG_SHORTS:
            s->REG.SHORTS = value;
            break;

        case O_RNG_INTEN:
            s->REG.INTEN = value;
            break;

        case O_RNG_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_RNG_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_RNG_CONFIG:
            s->REG.CONFIG = value;
            break;

        default:
            printf("[rng] unk write offset: %llx\n", offset);
            break;
    }
}

static void nrf51_timer_qtick(void * opaque)
{
    nrf51_timer_state * const s = opaque;
    if (!s->task_started)
    {
        return;
    }

#if TIMER_LOG_ENABLED
    const uint32_t mask = nrf51_timer_getmask(s);
    const int64_t clock_now = get_clock_virt() - s->clock_begin;
    const int64_t gclock_now = (clock_now - s->clock_begin) / s->ns_per_tick;
    const uint32_t gclock_masked = gclock_now & mask;

    printf("qtick\n");
    printf("gclock masked: %u\n", gclock_masked);
    printf("reg cc[0]: %u\n", s->REG.CC[0] & mask);
#endif

    /*
     * timer_tick function simulates each timer tick.
     * Since TIMER module works in two modes (counter and timer)
     * Timer mode is simulated by QEMU timers.
     * Counter mode only requires timer_tick function to be called.
     * It increments (s->tick) by one and checks for compare registers.
     * It is not feasible to call timer_tick function 16M times a second
     * for timer mode. So we only call it when it is required by calculating
     * next trigger time and with the help of QEMU timers.
     * We only give the previous value for tick, timer_tick increases the value
     * then does its job.
     * 
     * It is enough to pass last CC value used for the comparison.
     */
    s->tick = (s->cc_used - 1);
    nrf51_timer_tick(s);

    if (s->task_started)
    {
        s->clock_begin = get_clock_virt();
        nrf51_timer_set_next_event(s);
    }
}

static uint32_t nrf51_timer_getmask(nrf51_timer_state * s)
{
    static const uint32_t bitmode_mask[TIMER_NUM_CC_REGS] =
    {
        [0] = 0xFFFF,     //16 bit
        [1] = 0xFF,       //8 bit
        [2] = 0xFFFFFF,   //24 bit
        [3] = 0xFFFFFFFF  //32 bit
    };

    //printf("[TIMER%d] using BITMODE: %u\n", s->num_instance, (uint32_t) s->REG.BITMODE);
    g_assert((s->counter_max & 0xFF) == 0xFF);
    return bitmode_mask[s->REG.BITMODE & 0x3] & s->counter_max;
}

static int64_t nrf51_timer_set_next_event(nrf51_timer_state * s)
{
    const uint32_t mask = nrf51_timer_getmask(s);
    const uint32_t cc[TIMER_NUM_CC_REGS] =
    {
        //Mask and save each CC register.
        s->REG.CC[0] & mask,
        s->REG.CC[1] & mask,
        s->REG.CC[2] & mask,
        s->REG.CC[3] & mask,
    };

    g_assert(s->ns_per_tick > 0);
    //relative to the guest's timer clock
    //const int64_t clock_now = get_clock_virt();
    const int64_t gclock_now = s->tick; //(clock_now - s->clock_begin) / s->ns_per_tick;
    const uint32_t gclock_masked = gclock_now & mask;
    int64_t closest = ((uint32_t) -1);

    for (int i = 0; i < TIMER_NUM_CC_REGS; i++)
    {
        uint32_t dist;
        if (cc[i] > gclock_masked)
        {
            dist = cc[i] - gclock_masked;
        }
        else
        {
            dist = mask - gclock_masked + cc[i];
        }

        if (dist < closest)
        {
            closest = dist;
            s->cc_used = cc[i];
        }
    }

    const int64_t next_event_time = s->clock_begin + closest * s->ns_per_tick;

#if TIMER_LOG_ENABLED
    printf("[TIMER%d] using cc value: %u (mask 0x%x)\n",
            s->num_instance, s->cc_used, mask);
    printf("host clock begin: %lld\n", s->clock_begin);
    //printf("host clock now:   %lld\n", clock_now);
    printf("guest tick now:   %lld\n", gclock_now);
    printf("guest closest tick: %lld\n", closest);
    printf("next event time in ns: %lld\n", next_event_time);
    printf("[TIMER%d] closest event (ms): %lld\n",
           s->num_instance, (next_event_time - s->clock_begin) / SCALE_MS);
#endif

    timer_mod_ns(s->qtimer, next_event_time);

    return next_event_time;
}

static void nrf51_timer_tick(void * opaque)
{
    nrf51_timer_state * const s = opaque;

    if (!s->task_started)
    {
        /*
         * STOP task must have been triggered
         * or START task was never triggered.
         */
        return;
    }

    const uint32_t mask = nrf51_timer_getmask(s);
    ++s->tick;
    const uint32_t tick_masked = s->tick & mask;
    const uint32_t cc[TIMER_NUM_CC_REGS] =
    {
        //Mask and save each CC register.
        s->REG.CC[0] & mask,
        s->REG.CC[1] & mask,
        s->REG.CC[2] & mask,
        s->REG.CC[3] & mask,
    };

    bool pulse = false;

    //printf("tick: %u masked: %u\n", s->tick, tick_masked);

    for (int i = 0; i < TIMER_NUM_CC_REGS; i++)
    {
        if (cc[i] == tick_masked)
        {
#if TIMER_LOG_ENABLED
            printf("CC[%d]: %u, tick_masked: %u\n", i, cc[i], tick_masked);
#endif
            const uint8_t short_mask = (1 << i);
            s->REG.COMPARE[i] = 1; //Set event
#if TIMER_LOG_ENABLED
            printf("[TIMER%d] evt COMPARE[%d]\n", s->num_instance, i);
#endif
            ppi_event_filter(s->base + O_TIMER_COMPARE0 + i*4);

            //Check for CLEAR and STOP shortcuts
            if (s->REG.SHORTS & short_mask)
            {
#if TIMER_LOG_ENABLED
                printf("[TIMER%d] COMPARE -> CLEAR\n", s->num_instance);
#endif
                nrf51_timer_write(s, O_TIMER_CLEAR, 1, sizeof(uint32_t));
            }

            if ( (s->REG.SHORTS >> 4) & short_mask )
            {
#if TIMER_LOG_ENABLED
                printf("[TIMER%d] COMPARE -> STOP\n", s->num_instance);
#endif
                nrf51_timer_write(s, O_TIMER_STOP, 1, sizeof(uint32_t));
            }

            //Check for INTEN flag
            if ( (s->REG.INTEN >> 16) & short_mask )
            {
                pulse = true;
            }
        }
    }

    if (pulse)
        qemu_irq_pulse(s->irq);

}

static uint64_t nrf51_timer_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_timer_state *s = opaque;

    switch(offset)
    {
        case O_TIMER_SHORTS:    return s->REG.SHORTS;
        case O_TIMER_MODE:      return s->REG.MODE;
        case O_TIMER_BITMODE:   return s->REG.BITMODE;
        case O_TIMER_PRESCALER: return s->REG.PRESCALER;

        case O_TIMER_INTENSET:
        case O_TIMER_INTENCLR:
            return s->REG.INTEN;

        case O_TIMER_CC0 ... O_TIMER_CC3:
            return s->REG.CC[(offset - O_TIMER_CC0) / 4];

        //Events
        case O_TIMER_COMPARE0 ... O_TIMER_COMPARE3:
            return s->REG.COMPARE[(offset - O_TIMER_COMPARE0) / 4];

    }

    fprintf(stderr, "TIMER rd unknown reg: 0x%x\n", (uint32_t) offset);
    return (uint64_t)-1;
}

static void nrf51_timer_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_timer_state *s = opaque;
    //printf("[TIMER%d] wr 0x%x := 0x%x\n", s->num_instance, (uint32_t) offset, (uint32_t) value);
    switch(offset)
    {
        /* Tasks */
        case O_TIMER_START:
        {
            //By default prescaler is not used for COUNTER mode
            uint16_t prescaler = 0;
            s->task_started = true;

            if (s->REG.MODE != TIMER_MODE_TIMER)
            {
                printf("[TIMER%d] started in COUNTER mode\n", s->num_instance);
                break;
            }

            prescaler = s->REG.PRESCALER & 0xF;
            if (prescaler > 9)
            {
                prescaler = 9;
                //TODO: Check behavior on device.
                qemu_log_mask(LOG_GUEST_ERROR, "timer prescaler set to %u, using max (9)\n", prescaler);
            }
            prescaler = 1 << prescaler;
            const uint32_t guest_freq = FREQ_16MHZ / prescaler;
            s->ns_per_tick = get_ns_per_tick(guest_freq);
            s->clock_begin = get_clock_virt();
            const int64_t next_event_time = nrf51_timer_set_next_event(s);
#if TIMER_LOG_ENABLED
            printf("guest frequency: %u, ns/tick: %u\n", guest_freq, s->ns_per_tick);
            printf("next event time: %lld\n", next_event_time);
#else
            (void)next_event_time;
#endif
            break;
        }

        case O_TIMER_SHUTDOWN:
            s->tick = 0;
            //fall through
        case O_TIMER_STOP:
            s->task_started = false;
            timer_del(s->qtimer);
            break;

        case O_TIMER_COUNT:
            if (s->REG.MODE == TIMER_MODE_COUNTER && s->task_started)
            {
                nrf51_timer_tick(s);
            }
            break;

        case O_TIMER_CAPTURE0 ... O_TIMER_CAPTURE3:
        {
            g_assert(s->REG.MODE == TIMER_MODE_TIMER || s->REG.MODE == TIMER_MODE_COUNTER);
            const int cc_idx = (offset - O_TIMER_CAPTURE0) / 4;
            if (s->REG.MODE == TIMER_MODE_TIMER)
            {
                if (s->task_started)
                {
                    //Only capture the value if START task was triggered before.
                    g_assert(s->ns_per_tick > 0);
                    s->REG.CC[cc_idx] =
                        (get_clock_virt() / s->ns_per_tick) &
                        nrf51_timer_getmask(s);
                }
            }
            else
            {
                //Counter mode
                s->REG.CC[cc_idx] = s->tick;
                //printf("[TIMER] counter capture: %u\n", s->tick);
            }

            break;
        }

        case O_TIMER_CLEAR:
            s->tick = 0;
            if (s->REG.MODE == TIMER_MODE_TIMER)
            {
                if (s->task_started)
                {
                   /*
                    * It is needed to clear counter value but
                    * in timer mode we do not count ticks.
                    * Instead we use QEMU timers for performance
                    * concerns.
                    * So let's just restart the timer by this recursive call.
                    * This will also modify existing QEMU timer.
                    */
                    //printf("[TIMER%d] clear\n", s->num_instance);
                    timer_del(s->qtimer);
                    nrf51_timer_write(s, O_TIMER_START, 1, sizeof(uint32_t));
                }
            }
            break;

        /* Registers */
        case O_TIMER_SHORTS:
            s->REG.SHORTS = value;
            break;

        case O_TIMER_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_TIMER_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_TIMER_MODE:
            s->REG.MODE = value & 0x1;
            break;

        //TODO: unpredictible behavior on device if updated when timer is running.
        case O_TIMER_BITMODE:
            s->REG.BITMODE = value;
            break;

        //TODO: unpredictible behavior on device if updated when timer is running.
        case O_TIMER_PRESCALER:
            s->REG.PRESCALER = value;
            break;

        //TODO: Is this supposed to be RW?
        case O_TIMER_CC0 ... O_TIMER_CC3:
            //TODO: need to recalculate next timer event?
            s->REG.CC[(offset - O_TIMER_CC0) / 4] = value;
            if (s->task_started)
            {
                //Calculate the next event time if timer was started.
                (void) nrf51_timer_set_next_event(s);
            }
            break;

        //Events
        case O_TIMER_COMPARE0 ... O_TIMER_COMPARE3:
            s->REG.COMPARE[(offset - O_TIMER_COMPARE0) / 4] = value;
            break;

        default:
            fprintf(stderr, "TIMER unknown reg: 0x%x\n", (uint32_t) offset);
            break;
    }
}

static uint64_t nrf51_ficr_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    //nrf51_ficr_state *s = opaque;

    switch(offset)
    {
        case O_FICR_CODEPAGESIZE: return NRF51_CODEPAGESIZE;

        case O_FICR_CODESIZE: return NRF51_CODESIZE;

        case O_FICR_CLENR0:
            return (uint64_t) -1; //TODO: Check UICR CLENR0 config.

        case O_FICR_PPFC: return 0xFF; //Not present

        case O_FICR_NUMRAMBLOCK: return NUMRAMBLOCK; //4 blocks

        case O_FICR_SIZERAMBLOCK1: //Deprecated
        case O_FICR_SIZERAMBLOCK2: //Deprecated
        case O_FICR_SIZERAMBLOCK3: //Deprecated
        case O_FICR_SIZERAMBLOCKS:
            return SIZERAMBLOCKS;

        //Hardcoded, HWID = 0x86; FWID = deprecated.
        case O_FICR_CONFIGID: return 0xffff0086; 

        //Hardcoded Device ID.
        //FIXME: Use device id from command line
        case O_FICR_DEVICEID0: return DEVICEID_LO;
        case O_FICR_DEVICEID1: return DEVICEID_HI;

        case O_FICR_DEVICEADDR0:
        {
            static uint8_t r;
            while(!r)
            {
                r = xsrand();
            }
            return DEVICEADDR_LO + r;
        }
        case O_FICR_DEVICEADDR1: return DEVICEADDR_HI;

        case O_FICR_DEVICEADDRTYPE: return DEVICEADDRTYPE_RANDOM;

        //TODO: unsupported
        case O_FICR_ER0 ... O_FICR_ER3:
            return (uint64_t) -1;

    }

    printf("ficr, not implemented: 0x%llx\n", offset);

    return (uint64_t)-1;
}

static void nrf51_ficr_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{

}

static uint64_t nrf51_wdt_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_wdt_state * const s = opaque;

    switch (offset)
    {
        case O_WDT_INTENSET:
        case O_WDT_INTENCLR:
            return s->REG.INTEN;

        case O_WDT_RR0 ... O_WDT_RR7:
            return WDT_RELOAD_VALUE;

        case O_WDT_CRV:       return s->REG.CRV;
        case O_WDT_RUNSTATUS: return s->REG.RUNSTATUS;
        case O_WDT_REQSTATUS: return s->REG.REQSTATUS;
        case O_WDT_RREN:      return s->REG.RREN;

    }

    qemu_log_mask(LOG_GUEST_ERROR, "WDT: read at bad offset 0x%x\n", (uint32_t)offset);
    return (uint64_t)-1;
}

static void nrf51_wdt_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_wdt_state * const s = opaque;

    switch (offset)
    {
        case O_WDT_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_WDT_INTENCLR:
            s->REG.INTEN &= ~value;
            break;
        
        case O_WDT_RREN:
            if (!s->started)
            {
                s->REG.RREN = value;
            }
            break;

        case O_WDT_CRV:
            if (!s->started)
            {
                s->REG.CRV = value;
            }
            break;

        case O_WDT_CONFIG:
            if (!s->started)
            {
                s->REG.CONFIG = value;
            }
            break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                "WDT: write at bad offset 0x%x\n", (uint32_t)offset);
            break;
    }
}

static uint64_t nrf51_ccm_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_ccm_state *s = opaque;


    switch (offset)
    {
        case O_CCM_SCRATCHPTR: return s->REG.SCRATCHPTR;
        case O_CCM_CNFPTR    : return s->REG.CNFPTR;
        case O_CCM_INPTR     : return s->REG.INPTR;
        case O_CCM_OUTPTR    : return s->REG.OUTPTR;
        case O_CCM_ENDKSGEN  : return s->REG.ENDKSGEN;
        case O_CCM_ENDCRYPT  : return s->REG.ENDCRYPT;
        case O_CCM_ERROR     : return s->REG.ERROR;
        case O_CCM_ENABLE    : return s->REG.ENABLE;
        case O_CCM_MODE      : return s->REG.MODE;
        case O_CCM_MICSTATUS : return 1;//s->REG.MICSTATUS;


        case O_CCM_INTENSET:
        case O_CCM_INTENCLR:
            return s->REG.INTEN;

    default:
        break;
    }

    return (uint64_t)-1;
}

static inline uint8_t get_ccm_inptr_tot_len(uint8_t * pkt_ptr)
{
    return get_ccm_len(pkt_ptr) + 3; /* hdr + len + rfu */
}

static uint8_t * nrf51_ccm_ptr_to_ram(nrf51_ccm_state * s, uint32_t ptr, uint32_t minimum)
{
    uint8_t * const ram_ptr = memory_region_get_ram_ptr(&bootinfo.sram);
    uint8_t * pkt_ptr;
    if (ptr < NRF51_SRAM_BASE)
    {
        return NULL;
    }
    ptr -= NRF51_SRAM_BASE;
    if (ptr >= SRAM_32K)
    {
        return NULL;
    }

    pkt_ptr = ram_ptr + ptr;

    if (!minimum)
    {
        minimum = get_ccm_inptr_tot_len(pkt_ptr);
    }

    if (ptr + minimum >= SRAM_32K)
    {
        return NULL;
    }

    return pkt_ptr;
}

/*
 * Implemented according to Bluetooth Specification v4.0
 * Specification Volume 6 (Core System Package) / Part E / 2.1 CCM Nonce
 */
static bool nrf51_ccm_gen_nonce(nrf51_ccm_state * s)
{
    uint8_t * const ram_ptr = memory_region_get_ram_ptr(&bootinfo.sram);
    uint32_t cnfptr = s->REG.CNFPTR;
    if (cnfptr < NRF51_SRAM_BASE || (cnfptr & 0x3)) //Check that 32-bit aligned.
    {
        fprintf(stderr, "ccm cnfptr lower than sram base\n");
        return false;
    }
    cnfptr -= NRF51_SRAM_BASE;
    if (cnfptr > (SRAM_32K - sizeof(ccm_cnf_t)))
    {
        fprintf(stderr, "ccm cnfptr out of sram bounds\n");
        return false;
    }

    //FIXME: Reading cnf on different host platform might cause faults. (unaligned read)
    ccm_cnf_t * cnf = (void*)(ram_ptr + cnfptr);

    const uint8_t * pktctr = (uint8_t*) &cnf->pktctr;
    hexdump("pktctr", pktctr, 5);
    s->nonce[0] = pktctr[0];
    s->nonce[1] = pktctr[1];
    s->nonce[2] = pktctr[2];
    s->nonce[3] = pktctr[3];
    s->nonce[4] = pktctr[4] & 0x7f;
    s->nonce[4] |= cnf->direction_bit << 7;
#define CCM_IV_OFFSET (5) //FIXME: move
    memcpy(s->nonce + CCM_IV_OFFSET, cnf->iv, sizeof(s->nonce) - CCM_IV_OFFSET);
    memcpy(s->key, cnf->key, sizeof(s->key));
    QEMU_BUILD_BUG_ON(sizeof(s->nonce) - CCM_IV_OFFSET != sizeof(cnf->iv));

    hexdump("nonce", s->nonce, sizeof(s->nonce));
    return true;
}

static void nrf51_ccm_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    nrf51_ccm_state * s = opaque;
    CCMDP("wr 0x%x := 0x%x", (uint32_t) offset, (uint32_t) value);
    switch (offset)
    {
        case O_CCM_CRYPT:
            {
                bool error = false;
                uint8_t * inptr = nrf51_ccm_ptr_to_ram(s, s->REG.INPTR, 0);
                uint8_t * outptr = nrf51_ccm_ptr_to_ram(s, s->REG.OUTPTR, get_ccm_inptr_tot_len(inptr) + NRF51_MIC_SZ);
                uint8_t * sc_ptr = nrf51_ccm_ptr_to_ram(s, s->REG.SCRATCHPTR, AES_CCM_SCRATCH_SZ);
                if (inptr && outptr && sc_ptr)
                {
                    outptr[0] = get_ccm_hdr(inptr); //copy header
                    outptr[1] = inptr[1] + NRF51_MIC_SZ; //Adjust len
                    outptr[2] = 0; //RFU
                    uint8_t * mic_ptr = get_ccm_data_ptr(outptr) + get_ccm_len(inptr); //dataptr + inputlen
                    /*
                     * Note:
                     * CCM can authenticate unencrypted data, which is the additional authenticated data (AAD).
                     * In that case, a single byte header is calculated as AAD.
                     * It is transmitted unecrypted but is also authenticated. 
                     */
                    const int ret = aes_ccm_ae(s->key, AES_ECB_BLOCK_SZ, s->nonce, NRF51_MIC_SZ,
                                                get_ccm_data_ptr(inptr), get_ccm_len(inptr),
                                                inptr /* AAD */, 1 /* AAD Len */, get_ccm_data_ptr(outptr), mic_ptr);
                    if (!(error = ret))
                    {
                        //TODO: Check inten and irq pulse
                        s->REG.ENDCRYPT = 1;
                        printf("crypt done\n");
                    }
                }
                else
                {
                    error = true;
                }
                if (error)
                {
                    //TODO: gen irq (check inten)
                    s->REG.ERROR = 1;
                }
            }
            break;
        case O_CCM_KSGEN:
            if(nrf51_ccm_gen_nonce(s))
            {
                s->REG.ENDKSGEN = 1;
                //TODO: irq
                /*
                if (s->REG.INTEN & mask)
                {
                    qemu_irq_pulse(s->irq)
                }
                */
                //TODO: short
                /*
                if (s->REG.SHORTS & 0x1) //endksgen_crypt
                {
                    call crypt
                }
                */
            }
            else
            {
                //TODO: Trigger error event (check RM)
                s->REG.ERROR = 1;
            }
            break;

        case O_CCM_ENDKSGEN:
            s->REG.ENDKSGEN = value;
            break;
        case O_CCM_ENDCRYPT:
            s->REG.ENDCRYPT = value;
            break;
        case O_CCM_ERROR:
            s->REG.ERROR = value;
            break;
        case O_CCM_ENABLE:
            s->REG.ENABLE = value;
            break;
        case O_CCM_MODE:
            s->REG.MODE = value;
            break;
        case O_CCM_CNFPTR:
            s->REG.CNFPTR = value;
            break;
        case O_CCM_INPTR:
            s->REG.INPTR = value;
            break;
        case O_CCM_OUTPTR:
            s->REG.OUTPTR = value;
            break;
        case O_CCM_SCRATCHPTR:
            s->REG.SCRATCHPTR = value;
            break;
       case O_CCM_INTENSET:
            s->REG.INTEN |= value;
            break;
        case O_CCM_INTENCLR:
            s->REG.INTEN &= ~value;
            break;
        default:
            break;
    }
}

static uint64_t nrf51_ppi_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    //nrf51_ppi_state *s = opaque;

    switch (offset)
    {
        case O_PPI_CHEN:
        case O_PPI_CHENSET:
        case O_PPI_CHENCLR:
            return ppis.REG.CHEN;
    }

    qemu_log_mask(LOG_GUEST_ERROR, "PPI: unk read at 0x%x\n", (uint32_t)offset);
    return (uint64_t)-1;
}

static bool ppi_evt_compare(const void *obj, const void *userp)
{
    const ppi_event_info * const evinfo = obj;
    const uint32_t * const event_addr = userp;

    return (evinfo->addr == *event_addr);
}

static ppi_event_info * nrf51_ppi_find_event(uint32_t event_addr)
{
    return qht_lookup(ppis.event_map, ppi_evt_compare, &event_addr, event_addr);
}

static bool ppi_is_chan_enabled(uint8_t chan)
{
    const uint32_t chbit = (1<<chan);
    return !!(ppis.REG.CHEN & chbit); //check that ch. is enabled.
}

static void ppi_event_filter(uint32_t event_addr)
{
    ppi_event_info * evinfo = nrf51_ppi_find_event(event_addr);

    PPIDP("%s event: %s\n", evinfo ? "" : " ignore", eptostr(event_addr));

    if (event_addr < 0x4000000)
    {
        fprintf(stderr, "invalid event: 0x%x\n", event_addr);
        g_assert(event_addr >= 0x4000000);
        return;
    }

    if (evinfo)
    {
        g_assert(evinfo->ref_cnt > 0);
        for (int i = 0; i < evinfo->ref_cnt; i++)
        {
            const uint8_t chan = evinfo->channels[i];
            //Trigger the configured task for every channel, if enabled.
            if (ppi_is_chan_enabled(chan))
            {
                const uint32_t tep = ppis.REG.CH[chan].TEP;
                if (chan < 16)
                {
                    g_assert(event_addr == ppis.REG.CH[chan].EEP);
                }
                g_assert(tep > 0x40000000); //sanity check
                if (tep != 0x4000800c)
                {
                    PPIDP("trigger task: %s (from: %s)\n", eptostr(tep), eptostr(event_addr));
                }
                g_assert(tep); //FIXME: not a QEMU development error, use warning.
                uint32_t dummy = 1;
                if (event_addr == 0x40008140)
                {
                    PPIDP("TIMER0_COMPARE0 (ch: %u), TEP: %s\n", chan, eptostr(tep));
                }
                //This will call write op from the corresponding peripheral
                address_space_write(&address_space_memory, tep,
                                    MEMTXATTRS_UNSPECIFIED,
                                    (uint8_t*)&dummy, sizeof(dummy));
            }
            else if (chan < 16 || chan == 20)
            {
                PPIDP("Chan %u disabled, skip E/T: %s/%s\n",
                    chan, eptostr(event_addr), eptostr(ppis.REG.CH[chan].TEP));
            }
        }
    }
}

static void ppi_add_event(uint8_t chan, uint32_t eep, uint32_t tep)
{
    ppi_event_info * evinfo = nrf51_ppi_find_event(eep);
    if (!evinfo)
    {
        //This event was not configured before.
        evinfo = g_new0(ppi_event_info, 1);
        evinfo->addr = eep;
        qht_insert(ppis.event_map, evinfo, eep);
    }

    g_assert(evinfo);
    //Add this channel into the list of channels for this event.
    evinfo->channels[evinfo->ref_cnt++] = chan;
    ppis.REG.CH[chan].EEP = eep;
    PPIDP("add event %s at chan: %d (total chans for this event: %d)\n",
            eptostr(eep), chan, evinfo->ref_cnt);

    if (tep)
    {
        ppis.REG.CH[chan].TEP = tep;
        PPIDP(" task %s at chan: %d\n", eptostr(tep), chan);
    }
}

void nrf51_radio_event_ready_ppi(void)
{
    ppi_event_filter(RADIO_BASE + O_RADIO_READY);
}

static void nrf51_ppi_remove_chan_from_event(ppi_event_info * evinfo, uint8_t chan)
{
    g_assert(evinfo->ref_cnt > 0);
    g_assert(evinfo->ref_cnt <= (PPI_NUM_USER_CHANS + PPI_NUM_FIXED_CHANS));
    bool removed = false;

    //TODO: remove this?
    if (evinfo->ref_cnt > (PPI_NUM_USER_CHANS + PPI_NUM_FIXED_CHANS))
    {
        fprintf(stderr, "development error in PPI\n");
        return;
    }

    for (int i = 0; i < evinfo->ref_cnt; i++)
    {
        if (evinfo->channels[i] == chan)
        {
            //Replace the removed event with the last one in the array.
            //Then reduce the ref. count.
            const uint8_t last = evinfo->channels[evinfo->ref_cnt - 1];
            evinfo->channels[i] = last;
            evinfo->ref_cnt--;
            removed = true;
            break;
        }
    }

    //Check that this event was configured for the given channel previously.
    g_assert(removed);

    if (evinfo->ref_cnt <= 0)
    {
        qht_remove(ppis.event_map, evinfo, evinfo->addr);
        g_free(evinfo);
        return;
    }
}

static void nrf51_ppi_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned int size)
{
    //nrf51_ppi_state *s = opaque;

    switch (offset)
    {
        //Tasks
        case O_PPI_CHG_0_EN ... O_PPI_CHG_3_DIS:
        {
            offset = offset - O_PPI_CHG_0_EN;
            const int num_chg = offset / 8;
            if(!(offset & 0x4)) //mod 8
            {
                PPIDP("task chan group enable: %d\n", num_chg);
                //Enable channels in channel group n
                ppis.REG.CHEN |= ppis.REG.CHG[num_chg];
            }
            else
            {
                PPIDP("task chan group disable: %d\n", num_chg);
                //Disable channels in channel group n
                ppis.REG.CHEN &= ~ppis.REG.CHG[num_chg];
            }
            PPIDP("new CHEN: 0x%x\n", ppis.REG.CHEN);
            break;
        }

        //Registers
        case O_PPI_CHEN:
            ppis.REG.CHEN = value;
            PPIDP("CHEN 0x%x\n", ppis.REG.CHEN);
            break;

        case O_PPI_CHENSET:
            ppis.REG.CHEN |= value;
            PPIDP("CHEN 0x%x\n", ppis.REG.CHEN);
            break;

        case O_PPI_CHENCLR:
            ppis.REG.CHEN &= ~value;
            PPIDP("CHEN 0x%x\n", ppis.REG.CHEN);
            break;

        case O_PPI_CH_0_EEP ... O_PPI_CH_15_TEP:
        {
            offset = offset - O_PPI_CH_0_EEP;
            const int num_chan = offset / 8;
            g_assert(num_chan < 16);

            //TODO: warn user if any CHG is already enabled?

            if (offset & 0x4)
            {
                //Offset is TEP

                //FIXME: (Sanity check) Need to see if TEP points to a task.
                ppis.REG.CH[num_chan].TEP = value;
                PPIDP("Chan %d set task: 0x%x\n", num_chan, (uint32_t)value);
            }
            else
            {
                //Offset is EEP

                //FIXME: (Sanity check) Need to see if EEP points to an event.

                if (ppis.REG.CH[num_chan].EEP != value)
                {
                    if(ppis.REG.CH[num_chan].EEP)
                    {
                        //We MUST have a previous event configured in this channel.
                        ppi_event_info * old_evinfo = nrf51_ppi_find_event(ppis.REG.CH[num_chan].EEP);
                        g_assert(old_evinfo);
                        nrf51_ppi_remove_chan_from_event(old_evinfo, (uint8_t) num_chan);

                        //old_evinfo now may point to freed memory area and can not be used.
                        //assume old_evinfo = NULL;
                    }

                    ppi_add_event(num_chan, (uint32_t) value, 0x0);
                }
                else
                {
                    PPIDP("info: Same EEP written twice.\n");
                }
            }
        }
        break;

        case O_PPI_CHG_0 ... O_PPI_CHG_3:
        {
            int num_chg = (offset - O_PPI_CHG_0) / 4;
            ppis.REG.CHG[num_chg] = value;
            PPIDP("CHG[%d] = 0x%x\n", num_chg, (uint32_t) value);
        }
        break;

        default:
            qemu_log_mask(LOG_GUEST_ERROR, "PPI: unk write at 0x%x\n", (uint32_t)offset);
    }
}

static void nrf51_ecb_init(Object *obj)
{
    nrf51_ecb_state *s = NRF51_ECB_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_ecb_ops, s,
                          "mod-aes-ecb", ECB_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_ECB);

}

static void nrf51_gpio_init(Object *obj)
{
    //DeviceState *dev = DEVICE(obj);
    nrf51_gpio_state *s = NRF51_GPIO_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    g_gpio_state = s;

    memory_region_init_io(&s->iomem, obj, &nrf51_gpio_ops, s,
                          "mod-gpio", GPIO_REG_SPACE);
    sysbus_init_mmio(sbd, &s->iomem);

    packet_handler_context[PROTO_GPIO] = s;
}

static void nrf51_gpte_init(Object *obj)
{
    nrf51_gpte_state *s = NRF51_GPTE_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    //QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_gpte_ops, s,
                          "mod-gpiote", GPTE_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_GPTE);

    //bh = qemu_bh_new(..., s);
    //s->ptimer = ptimer_init(bh, TIMER_POLICY_DEFAULT);

}

static void nrf51_clock_init(Object *obj)
{
    nrf51_clock_state *s = NRF51_CLOCK_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    //QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_clock_ops, s,
                          "mod-clock", POWER_CLOCK_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_POWER_CLOCK);

    //bh = qemu_bh_new(..., s);
    //s->ptimer = ptimer_init(bh, TIMER_POLICY_DEFAULT);
}

/*
 * Called for each RTC module
 */
static void nrf51_rtc_init(Object *obj)
{
    static int num_rtc_instance;
    static char dev_name[] = "mod-rtcX";
    nrf51_rtc_state *s = NRF51_RTC_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    dev_name[sizeof(dev_name) - 2] = '0' + num_rtc_instance;
    s->num_instance = num_rtc_instance;

    memory_region_init_io(&s->iomem, obj, &nrf51_rtc_ops, s,
                          dev_name, RTC_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->base = (uint32_t) RTC_BASE_IRQ_LIST[num_rtc_instance].base;
    s->irq = qdev_get_gpio_in(nvic, RTC_BASE_IRQ_LIST[num_rtc_instance].irq);

    //Periodic timer to track RTC's counter value.
    s->qtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nrf51_rtc_timer, s);
    s->bRunning = false;

    qemu_mutex_init(&s->mtx);
    num_rtc_instance++;
}

static void nrf51_adc_init(Object *obj)
{
    nrf51_adc_state *s = NRF51_ADC_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_adc_ops, s,
                          "mod-adc", ADC_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_ADC);

    //A timer to simulate ADC conversion.
    bh = qemu_bh_new(nrf51_adc_conversion_complete, s);
    s->pt_conversion = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

    s->bConversionActive = false;

    //0 = No noise,
    //n+1 = n bits noise
    s->nNoiseBits = 4 + 1;
}

static void nrf51_uart_init(Object *obj)
{
    nrf51_uart_state *s = NRF51_UART_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_uart_ops, s,
                          "mod-uart", UART_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_UART);

    //A timer to simulate uart speeds and events.
    bh = qemu_bh_new(nrf51_uart_timer, s);
    s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

    //Initialize UNIX socket
    nrf51_uart_comm_init();
}

static void nrf51_ficr_init(Object *obj)
{
    nrf51_ficr_state *s = NRF51_FICR_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_ficr_ops, s,
                          "mod-ficr", FICR_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);
}

static void nrf51_uicr_save_regs(nrf51_uicr_state * s)
{
    /*
     * Registers are saved to file as they are.
     * Using the same save file on a host that has
     * different byte order is not suggested.
     */
    FILE * regfile = fopen(UICR_REG_FILE, "w");
    bool ok = false;

    if (regfile)
    {
        ok = (fwrite(&s->REG, sizeof(s->REG), 1, regfile) == 1);
        fclose(regfile);
        regfile = NULL;
    }

    if (ok)
    {
        fprintf(stderr, "Saved UICR regs\n");
    }
    else
    {
        fprintf(stderr, "error: cannot save UICR registers\n");
    }
}

static void nrf51_uicr_load_regs(nrf51_uicr_state * s)
{
    FILE * regfile = fopen(UICR_REG_FILE, "r+b");
    bool use_defaults = true;

    if (regfile)
    {
        use_defaults = (fread(&s->REG, sizeof(s->REG), 1, regfile) != 1);
        fclose(regfile);
        regfile = NULL;
    }

    if (use_defaults)
    {
        fprintf(stderr, "warning: using default UICR values\n");
        memset(&s->REG, 0xFF, sizeof(s->REG));
    }
}

static void nrf51_uicr_init(Object *obj)
{
    nrf51_uicr_state *s = NRF51_UICR_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_uicr_ops, s,
                          "mod-uicr", UICR_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    nrf51_uicr_load_regs(s);
}

void _nrf51_trigger_hardfault(const char * file, int line)
{
    printf("FIXME: trigger HardFault, %s:%d\n", file, line);
    fflush(stdout);
    exit(-1);
    //for(;;);
}

static void nrf51_ecb_class_init(ObjectClass *class, void *data)
{

    if(!qcrypto_cipher_supports(QCRYPTO_CIPHER_ALG_AES_128, QCRYPTO_CIPHER_MODE_ECB))
    {
        fprintf(stderr, "NRF51 requires AES128 cipher support\n");
        /*
         * FIXME:
         * 2. choices:
         * - Set a flag here and do not init. ecb module.
         * - Init. ECB module but guest won't be able
         *   to get result, instead ERRORECB will be
         *   triggered.
         */
    }
}

static void nrf51_radio_init(Object *obj)
{
    nrf51_radio_state *s = NRF51_RADIO_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH *bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_radio_ops, s,
                          "mod-radio", RADIO_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_RADIO);

    bh = qemu_bh_new(nrf51_radio_timer, s);
    ////s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

    {
        extern void set_irq_17_ctx(void * opaque);
        set_irq_17_ctx(s);
    }

    s->REG.POWER = 1;
    s->REG.FREQUENCY = 2;

    packet_handler_context[PROTO_RADIO] = s;

    nrf51_radio_udp_init(s);
}

static void nrf51_timer_init(Object *obj)
{
    static int num_timer_instance;
    static char dev_name[] = "mod-timerX";
    nrf51_timer_state *s = NRF51_TIMER_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    //replace 'X' with instance index.
    dev_name[sizeof(dev_name) - 2] = '0' + num_timer_instance;

    memory_region_init_io(&s->iomem, obj, &nrf51_timer_ops, s,
                          dev_name, TIMER_REG_SPACE); /*FIXME: use mod-timer0,1,2 */

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, TIMER_CONFIG[num_timer_instance].irq);

    s->base = (uint32_t)TIMER_CONFIG[num_timer_instance].base;
    s->counter_max = TIMER_CONFIG[num_timer_instance].counter_max;

    s->num_instance = num_timer_instance;

    s->REG.PRESCALER = 4; //Value on reset

    s->qtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nrf51_timer_qtick, s);

    num_timer_instance++;

}

static void nrf51_wdt_init(Object *obj)
{
    nrf51_wdt_state *s = NRF51_WDT_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_wdt_ops, s,
                          "mod-wdt", WDT_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    //Default values on reset
    s->REG.RREN = s->REG.CONFIG = s->REG.REQSTATUS = 1;
    s->REG.CRV = (uint32_t) -1;
}

static void nrf51_ppi_init(Object *obj)
{
    nrf51_ppi_state *s = NRF51_PPI_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    memset(&ppis, 0x00, sizeof(ppis));
    ppis.event_map = g_new0(struct qht, 1);

    //Initialize event to task map.
    //There are 16 configurable and 12 fixed channels.
    //We can have maximum 28 events configured,
    //32 elements seems like a reasonable number.
    qht_init(ppis.event_map, 32, QHT_MODE_AUTO_RESIZE);

    ppi_add_event(20, TIMER0_BASE + O_TIMER_COMPARE0, RADIO_BASE + O_RADIO_TXEN);
    ppi_add_event(21, TIMER0_BASE + O_TIMER_COMPARE0, RADIO_BASE + O_RADIO_RXEN);
    ppi_add_event(22, TIMER0_BASE + O_TIMER_COMPARE1, RADIO_BASE + O_RADIO_DISABLE);
    //ppi_add_event(23, RADIO_BASE + O_RADIO_BCMATCH, AAR TASK START);
    ppi_add_event(24, RADIO_BASE + O_RADIO_READY, CCM_BASE + O_CCM_KSGEN);
    ppi_add_event(25, RADIO_BASE + O_RADIO_ADDRESS, CCM_BASE + O_CCM_CRYPT);
    ppi_add_event(26, RADIO_BASE + O_RADIO_ADDRESS, TIMER0_BASE + O_TIMER_CAPTURE1);
    ppi_add_event(27, RADIO_BASE + O_RADIO_END, TIMER0_BASE + O_TIMER_CAPTURE2);
    ppi_add_event(28, RTC0_BASE + O_RTC_COMPARE0, RADIO_BASE + O_RADIO_TXEN);
    ppi_add_event(29, RTC0_BASE + O_RTC_COMPARE0, RADIO_BASE + O_RADIO_RXEN);
    ppi_add_event(30, RTC0_BASE + O_RTC_COMPARE0, TIMER0_BASE + O_TIMER_CLEAR);
    ppi_add_event(31, RTC0_BASE + O_RTC_COMPARE0, TIMER0_BASE + O_TIMER_START);

    memory_region_init_io(&s->iomem, obj, &nrf51_ppi_ops, s,
                          "mod-ppi", PPI_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);
}

static void nrf51_ccm_init(Object *obj)
{
    nrf51_ccm_state *s = NRF51_CCM_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_ccm_ops, s,
                          "mod-ccm", CCM_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

}

static void nrf51_rng_random_cb(void * opaque)
{
    nrf51_rng_state * s = opaque;
    uint32_t val = xsrand();

    val = val ^ (val >> 16);
    val = val ^ (val >> 8);

    s->REG.VALRDY = 1;
    s->REG.VALUE = val & 0xFF;

    if (s->REG.INTEN & 0x1)
    {
        qemu_irq_pulse(s->irq);
    }

    if ( s->REG.SHORTS & 0x1 )
    {
        ptimer_stop(s->ptimer);
    }
}

static void nrf51_rng_init(Object *obj)
{
    nrf51_rng_state *s = NRF51_RNG_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    QEMUBH * bh;

    INIT_DEV_STATE(s);

    memory_region_init_io(&s->iomem, obj, &nrf51_rng_ops, s,
                          "mod-rng", RNG_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    s->irq = qdev_get_gpio_in(nvic, IRQ_RNG);

    /*
     * Instead of generating random number
     * right after START task is triggered
     * it will be generated by a timer
     * so there will be a small delay.
     */
    bh = qemu_bh_new(nrf51_rng_random_cb, s);
    s->ptimer = ptimer_init(bh, PTIMER_POLICY_DEFAULT);

}

static void nrf51_nvm_init(Object *obj)
{
    nrf51_nvm_state *s = NRF51_NVM_STATE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    INIT_DEV_STATE(s);

    MemoryRegion * const sysmem = get_system_memory();

    memory_region_init_io(&s->iomem, obj, &nrf51_nvm_ops, s,
                          "mod-nvm", NVM_REG_SPACE);

    sysbus_init_mmio(sbd, &s->iomem);

    //Moving this to class level init/finalize would be better instead of instance.
    memory_region_init_rom_device(&s->flash, NULL, &nrf51_nvm_data_ops, s, "nrf51_flash", NRF51_FLASH_SIZE_IN_BYTES, &error_fatal);

    memory_region_add_subregion(sysmem, NRF51_FLASH_ORIGIN, &s->flash); //Subregion at offset 0x0
}

static uint64_t nrf51_ecb_read(void *opaque, hwaddr offset,
                          unsigned int size)
{
    nrf51_ecb_state *s = opaque;

    switch(offset)
    {
        case O_ECB_ENDECB:     return s->REG.ENDECB;
        case O_ECB_ERRORECB:   return s->REG.ERRORECB;
        case O_ECB_ECBDATAPTR: return s->REG.ECBDATAPTR;

        case O_ECB_INTENSET:
        case O_ECB_INTENCLR:
            /* 
             * TODO:
             * This register doesn't have base address
             * in ref. manual. However, it mentions
             * existence of it. It might be internal.
             */
            return s->REG.INTEN;
    }

    printf("[ecb_read] undefined register: 0x%lx\n", (unsigned long) offset);

    return (uint64_t)-1;
}

static void nrf51_ecb_error(nrf51_ecb_state *s)
{
    s->REG.ERRORECB = 1;
    if (s->REG.INTEN & ECB_ERRORECB_MASK) {
        qemu_irq_pulse(s->irq);
    }
}

static void nrf51_ecb_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    nrf51_ecb_state *s = opaque;
    MemTxResult res;
    Error *err;

    switch(offset)
    {
        //Tasks
        case O_ECB_STARTECB:
            if(NRF51_OUT_OF_RAM(s->REG.ECBDATAPTR, AES_ECB_HDR_SZ))
            {
                nrf51_trigger_hardfault();
                break;
            }
            //Fetch AES data from guest.
            res = address_space_read(&address_space_memory,
                    s->REG.ECBDATAPTR,
                    MEMTXATTRS_UNSPECIFIED,
                    s->ecb_data.key,
                    AES_ECB_READ_SZ); //Only read KEY and CLEARTEXT
            if (res != MEMTX_OK)
            {
                printf("ecb mem read error\n");
                nrf51_ecb_error(s);
                break;
            }

            if(s->cipher_ctx)
            {
                /*
                 * Compare current key and obtained key.
                 */
                if(memcmp(s->current_key, s->ecb_data.key, AES_ECB_BLOCK_SZ)){
                    //Key changed
                    qcrypto_cipher_free(s->cipher_ctx);
                    s->cipher_ctx = NULL;
                }
            }

            /*
             * Note:
             * if(s->cipher_ctx) and if(!s->cipher_ctx)
             * conditions can be fulfilled in the same
             * execution path.
             */

            if (!s->cipher_ctx){
                memcpy(s->current_key, s->ecb_data.key, sizeof(s->current_key));
                s->cipher_ctx = qcrypto_cipher_new(
                                    QCRYPTO_CIPHER_ALG_AES_128,
                                    QCRYPTO_CIPHER_MODE_ECB,
                                    s->current_key,
                                    G_N_ELEMENTS(s->current_key),
                                    &err);
                if(!s->cipher_ctx){
                    printf("[ecb] Cannot create cipher context\n");
                    nrf51_ecb_error(s);
                    break;
                }
            }

            //At this point we will always have cipher_ctx
            if (qcrypto_cipher_encrypt(s->cipher_ctx, s->ecb_data.cleartext,
                        s->ecb_data.ciphertext,
                        G_N_ELEMENTS(s->current_key),
                        &err) < 0){
                    printf("[ecb] encrypt returned error\n");
                    nrf51_ecb_error(s);
                    break;
            }

            //All good, put it into guest RAM.
            address_space_write(&address_space_memory,
                                s->REG.ECBDATAPTR + AES_ECB_CIPHERTEXT_OFFSET,
                                MEMTXATTRS_UNSPECIFIED, s->ecb_data.ciphertext,
                                AES_ECB_BLOCK_SZ);
            s->REG.ENDECB = 1;
            if(s->REG.INTEN & ECB_ENDECB_MASK){
                qemu_irq_pulse(s->irq);
            }
            break;

        case O_ECB_STOPECB:
            /*
             * It is not much possible to abort
             * ECB operation because QEMU doesn't
             * return to event loop until register
             * rd/wr function is returned. In that
             * case ECB operation will already be
             * finished when guest tries to write
             * into this register. So we ignore it.
             * FIXME: Check if ERRORECB is triggered
             * when AES operation is not started.
             */
            break;

        //Events
        case O_ECB_ENDECB:
            s->REG.ENDECB = value;
            break;

        case O_ECB_ERRORECB:
            s->REG.ERRORECB = value;
            break;

        //Registers
        case O_ECB_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_ECB_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_ECB_ECBDATAPTR:
            s->REG.ECBDATAPTR = value;
            break;

        default:
            printf("[ecb_write] undefined register: 0x%lx\n",
                (unsigned long) offset);
            break;
    }

}

#pragma endregion //Implementation

#pragma region ClassInit

static void nrf51_clock_class_init(ObjectClass *class, void *data){}
static void nrf51_gpio_class_init(ObjectClass *class, void *data){}
static void nrf51_gpte_class_init(ObjectClass *class, void *data){}
static void nrf51_rtc_class_init(ObjectClass *class, void *data){}
static void nrf51_adc_class_init(ObjectClass *class, void *data){}
static void nrf51_uart_class_init(ObjectClass *class, void *data){}
static void nrf51_ficr_class_init(ObjectClass *class, void *data){}
static void nrf51_radio_class_init(ObjectClass *class, void *data){}
static void nrf51_timer_class_init(ObjectClass *class, void *data){}
static void nrf51_rng_class_init(ObjectClass *class, void *data){}
static void nrf51_nvm_class_init(ObjectClass *class, void *data){}
static void nrf51_uicr_class_init(ObjectClass *class, void *data){}
static void nrf51_wdt_class_init(ObjectClass *class, void *data){}
static void nrf51_ppi_class_init(ObjectClass *class, void *data){}
static void nrf51_ccm_class_init(ObjectClass *class, void *data){}

#pragma endregion //ClassInit

#pragma region QEMU_CALLS
/*************************************\
 * QEMU Specific Calls
\*************************************/
static void nrf51_register_types(void)
{
    type_register_static(&nrf51_mod_adc);
    type_register_static(&nrf51_mod_clock);
    type_register_static(&nrf51_mod_ecb);
    type_register_static(&nrf51_mod_gpio);
    type_register_static(&nrf51_mod_gpte);
    type_register_static(&nrf51_mod_radio);
    type_register_static(&nrf51_mod_rng);
    type_register_static(&nrf51_mod_rtc);
    type_register_static(&nrf51_mod_timer);
    type_register_static(&nrf51_mod_uart);
    type_register_static(&nrf51_mod_nvm);
    type_register_static(&nrf51_mod_ficr);
    type_register_static(&nrf51_mod_uicr);
    type_register_static(&nrf51_mod_wdt);
    type_register_static(&nrf51_mod_ppi);
    type_register_static(&nrf51_mod_ccm);
}

type_init(nrf51_register_types)

DEFINE_MACHINE("nrf51", nrf51_machine_init)
#pragma endregion

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
