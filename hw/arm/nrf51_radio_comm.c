#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/arm/arm.h"
#include "exec/address-spaces.h"
#include "qapi/error.h"
#include "hw/misc/unimp.h"
#include "hw/ptimer.h"
#include "sysemu/sysemu.h"
#include "nrf51.h"
#include "nrf51_radio.h"
#include "sys/socket.h"
#include "nrf51_helper.h"

/* NOTES From Ref Manual *//*

17.1.3 Maximum packet length
Independent of the configuration of MAXLEN, the combined length of S0, LENGTH, S1 and PAYLOAD cannot exceed 254 bytes.

*//* NOTES END */

/* Defines */

//FIXME: Implement all shorts
#define SHORT_READY_START(s) ((s)->REG.SHORTS & 0x1)
#define SHORT_END_DISABLE(s) ((s)->REG.SHORTS & 0x2)
#define SHORT_DISABLED_TXEN(s) ((s)->REG.SHORTS & (1 << 2))
#define SHORT_DISABLED_RXEN(s) ((s)->REG.SHORTS & (1 << 3))
#define SHORT_END_START(s) ((s)->REG.SHORTS & (1 << 5))

/* Types */
typedef void(*state_func_t)(nrf51_radio_state*s);

/* Static Functions */
static void nrf51_radio_save_conf(nrf51_radio_state *s);
static void nrf51_radio_timer_start(nrf51_radio_state *s, bool run_first);
static void nrf51_radio_timer_stop(nrf51_radio_state * s);
static const char * task_to_str(radio_task_t task);
static const char * state_to_str(NRF51_RADIO_STATE state);
static void nrf51_radio_task_txen(nrf51_radio_state *s);
static void nrf51_radio_task_rxen(nrf51_radio_state *s);
static int nrf51_radio_udp_send(nrf51_radio_state * s, nrf51_air_packet_buff * pkt);
static void nrf51_radio_pulse_event(const nrf51_radio_state * s, uint32_t mask);
static void nrf51_radio_event_loop(void *opaque);

static bool fire_event_disabled(nrf51_radio_state *s);
static void state_disabled(nrf51_radio_state *s);
static void state_rxtxru(nrf51_radio_state *s);
static void state_rx(nrf51_radio_state *s);
static void state_tx(nrf51_radio_state *s);
static void state_rxtxidle(nrf51_radio_state *s);
static void state_rxtxdisable(nrf51_radio_state *s);
static void check_pending_task(nrf51_radio_state * const s);

/* Static Globals */
static nrf51_air_packet_buff air_packet;
static const state_func_t state_fns[] =
{
    [enmStateDisabled] = state_disabled,
    [enmStateRxRu] = state_rxtxru,
    [enmStateRxIdle] = state_rxtxidle,
    [enmStateRx] = state_rx,
    [enmStateRxDisable] = state_rxtxdisable,
    [enmStateTxRu] = state_rxtxru,
    [enmStateTxIdle] = state_rxtxidle,
    [enmStateTx] = state_tx,
    [enmStateTxDisable] = state_rxtxdisable,
};
static QemuMutex mtx_console;
static QemuMutex mtx_radio;

static void nrf51_radio_reset(nrf51_radio_state *s)
{
    //Reset anything starting from radio_state field up to REG.
    memset(&s->radio_state, 0x00,
            offsetof(nrf51_radio_state, REG) - offsetof(nrf51_radio_state, radio_state));
    memset(&s->REG, 0x00, sizeof(s->REG));

    s->REG.FREQUENCY = 2;
    s->REG.DATAWHITEIV = (1<<6); //FIXME: Bit-6 is hardwired to 1!!!!
    nrf51_radio_timer_stop(s);
}

/*
 * read op shall only be called by QEMU.
 * It is protected by mutex.
 */
uint64_t nrf51_radio_read(void *opaque, hwaddr offset,
                          unsigned size)
{
    nrf51_radio_state *s = opaque;

    RAD_LOCK();

    RDP("rd [%20s] 0x%x", regtostr((uint32_t)offset), (uint32_t) offset);

    if (offset == O_RADIO_READY) RDP("ready: %u\n", s->REG.READY);

    switch(offset)
    {
        /* Events */
        case O_RADIO_READY:     RAD_UNLOCK_RET(s->REG.READY);
        case O_RADIO_END:       RAD_UNLOCK_RET(s->REG.END);
        case O_RADIO_PAYLOAD:   RAD_UNLOCK_RET(s->REG.PAYLOAD);
        case O_RADIO_ADDRESS:   RAD_UNLOCK_RET(s->REG.ADDRESS);
        case O_RADIO_DISABLED:  RAD_UNLOCK_RET(s->REG.DISABLED);
        case O_RADIO_DEVMATCH:  RAD_UNLOCK_RET(s->REG.DEVMATCH);
        case O_RADIO_DEVMISS:   RAD_UNLOCK_RET(s->REG.DEVMISS);
        case O_RADIO_RSSIEND:   RAD_UNLOCK_RET(1);//s->REG.RSSIEND;
        case O_RADIO_BCMATCH:   RAD_UNLOCK_RET(s->REG.BCMATCH);

        /* Registers */
        case O_RADIO_INTENSET:
        case O_RADIO_INTENCLR:
            RAD_UNLOCK_RET(s->REG.INTEN);

        case O_RADIO_STATE:     RAD_UNLOCK_RET(s->radio_state); //TODO: Use REG.STATE?
        case O_RADIO_TIFS:      RAD_UNLOCK_RET(s->REG.TIFS); //Has no effect in QEMU.
        case O_RADIO_SHORTS:    RAD_UNLOCK_RET(s->REG.SHORTS);
        case O_RADIO_CRCSTATUS: RAD_UNLOCK_RET(s->REG.CRCSTATUS);
        case O_RADIO_CRCCNF:    RAD_UNLOCK_RET(s->REG.CRCCNF);
        case O_RADIO_RXMATCH:   RAD_UNLOCK_RET(s->REG.RXMATCH);
        case O_RADIO_RXCRC:     RAD_UNLOCK_RET(s->REG.RXCRC);
        case O_RADIO_PACKETPTR: RAD_UNLOCK_RET(s->REG.PACKETPTR);
        case O_RADIO_FREQUENCY: RAD_UNLOCK_RET(s->REG.FREQUENCY);
        case O_RADIO_POWER:     RAD_UNLOCK_RET(s->REG.POWER);
        case O_RADIO_PCNF0:     RAD_UNLOCK_RET(s->REG.PCNF0);
        case O_RADIO_PCNF1:     RAD_UNLOCK_RET(s->REG.PCNF1);
        case O_RADIO_TXPOWER:   RAD_UNLOCK_RET(s->REG.TXPOWER);
        case O_RADIO_MODE:      RAD_UNLOCK_RET(s->REG.MODE);
        case O_RADIO_BASE0:     RAD_UNLOCK_RET(s->REG.BASE0);
        case O_RADIO_BASE1:     RAD_UNLOCK_RET(s->REG.BASE1);
        case O_RADIO_RXADDRESSES: RAD_UNLOCK_RET(s->REG.RXADDRESSES);
        case O_RADIO_DATAWHITEIV: RAD_UNLOCK_RET(s->REG.DATAWHITEIV);
        case O_RADIO_BCC:       RAD_UNLOCK_RET(s->REG.BCC);
        case O_RADIO_DACNF:     RAD_UNLOCK_RET(s->REG.DACNF);
        case O_RADIO_DAI:       RAD_UNLOCK_RET(s->REG.DAI);
        case O_RADIO_RSSISAMPLE: RAD_UNLOCK_RET(67);
    }

    RDP("RD not implemented: 0x%x", (unsigned int) offset);
    RAD_UNLOCK();
    return (uint64_t) -1;
}

/*
 * write op shall only be called by QEMU.
 * It is protected by mutex.
 */
void nrf51_radio_write(void *opaque, hwaddr offset,
                       uint64_t value, unsigned size)
{
    nrf51_radio_state *s = opaque;

    RAD_LOCK();

    switch (offset)
    {
        /* Events */
        case O_RADIO_END:       s->REG.END      = value; break;
        case O_RADIO_READY:     s->REG.READY    = value; break;
        case O_RADIO_PAYLOAD:   s->REG.PAYLOAD  = value; break;
        case O_RADIO_ADDRESS:   s->REG.ADDRESS  = value; break;
        case O_RADIO_DEVMISS:   s->REG.DEVMISS  = value; break;
        case O_RADIO_RSSIEND:   s->REG.RSSIEND  = value; break;
        case O_RADIO_DISABLED:  s->REG.DISABLED = value; break;
        case O_RADIO_DEVMATCH:  s->REG.DEVMATCH = value; break;
        case O_RADIO_BCMATCH:   s->REG.BCMATCH  = value; break;

        /* Registers */
        case O_RADIO_INTENSET:
            s->REG.INTEN |= value;
            break;

        case O_RADIO_INTENCLR:
            s->REG.INTEN &= ~value;
            break;

        case O_RADIO_PACKETPTR:
            //TODO: check that ptr is in Data RAM otherwise give HardFault.
            //Check 5.1 for memory regions.
            //Byte aligned RAM address.
            s->REG.PACKETPTR = value;
            break;

        case O_RADIO_FREQUENCY:
            //TODO: Allowed value is [0..100]. Need to check?
            if (value > 100)
                printf("error: frequency value too big: %llu", value);
            s->REG.FREQUENCY = value;
            RDP("set freq: %u", (unsigned int) value + 2400);
            break;

        case O_RADIO_POWER:
            if ( !(value & 0x1) )
            {
                //Reset device state.
                nrf51_radio_reset(s);
            }
            s->REG.POWER = !!value;
            break;

        case O_RADIO_SHORTS:
            s->REG.SHORTS = value;
            break;

        case O_RADIO_PCNF0:
            s->REG.PCNF0 = value;
            break;

        case O_RADIO_PCNF1:
            s->REG.PCNF1 = value;
            //Other configuration values are acquired during START task.
            s->ActiveConf.PCNF1.MAXLEN = PCNF1_MAXLEN_READ(s->REG.PCNF1);
            s->ActiveConf.PCNF1.WHITEEN = PCNF1_WHITEEN_READ(s->REG.PCNF1);
            break;

        case O_RADIO_TXPOWER:
            //Ineffective in QEMU.
            s->REG.TXPOWER = value;
            break;

        case O_RADIO_MODE:
            s->REG.MODE = value & 0x3;
            break;

        case O_RADIO_PREFIX0:
            s->REG.PREFIX0 = value;
            break;

        case O_RADIO_PREFIX1:
            s->REG.PREFIX1 = value;
            break;

        case O_RADIO_BASE0:
            s->REG.BASE0 = value;
            break;

        case O_RADIO_BASE1:
            s->REG.BASE1 = value;
            break;

        case O_RADIO_TXADDRESS:
            s->REG.TXADDRESS = value & 0x7;
            break;

        case O_RADIO_RXADDRESSES:
            s->REG.RXADDRESSES = value & 0xff;
            break;

        case O_RADIO_CRCCNF:
            s->REG.CRCCNF = value & 0x1ff; //TODO: Do bit mask? What is the device behavior?
            break;

        case O_RADIO_CRCINIT:
            s->REG.CRCINIT = value & 0xFFFFFF;
            break;

        case O_RADIO_CRCPOLY:
            s->REG.CRCPOLY = value & 0xFFFFFF;
            break;

        case O_RADIO_TIFS: //Has no effect in QEMU.
            s->REG.TIFS = value;
            break;

        case O_RADIO_DATAWHITEIV:
            s->REG.DATAWHITEIV = (1<<6) | (value & 0x3f);
            break;

        case O_RADIO_BCC:
            s->REG.BCC = value;
            break;

        case O_RADIO_DACNF:
            s->REG.DACNF = value;
            break;

        /* Tasks */
        case O_RADIO_TXEN:
            if (value) //TODO: check for == 1 or any value?
                nrf51_radio_task_txen(s);
            break;

        case O_RADIO_RXEN:
            if (value) //TODO: check for == 1 or any value?
                nrf51_radio_task_rxen(s);
            break;

        case O_RADIO_START:
            if (value)
            {
                g_assert(s->radio_task == enmTaskNone);
                s->radio_task = enmTaskStart;
                nrf51_radio_timer_start(s, true);
                //TODO: call timer func and return as well?
            }
            break;

        case O_RADIO_DISABLE:
            //if (s->radio_state == enmStateRxDisable || s->radio_state == enmStateTxDisable)
            if (s->radio_task != enmTaskNone && s->radio_task != enmTaskDisable)
            {
                RDP("error: a task is already running: %s", task_to_str(s->radio_task));
                g_assert(!"EXIT!");
            }
            s->radio_task = enmTaskDisable;
            //Run event loop and immediately disable radio
            nrf51_radio_timer_start(s, true);
            break;
//TODO: check for break; in switch statements in entire project.
        default:
            printf("[radio] WR not implemented: 0x%x\n", (unsigned int) offset);
            break;
    }

    RAD_UNLOCK();
}

static void nrf51_radio_task_txen(nrf51_radio_state *s)
{
    RDP("%s", "Trigger TXEN");
    //FIXME: get value in state handler (timer).
    s->ActiveConf.uFrequency = s->REG.FREQUENCY & 0x7f;
    //TODO: what is the decision point for MODE?
    s->ActiveConf.uMode = s->REG.MODE;

    if (s->radio_state != enmStateDisabled)
    {
        PRINT_INCORRECT_STATE();
    }

    if (s->radio_task != enmTaskNone)
    {
        RDP("error: a task is already running: %s", task_to_str(s->radio_task));
        g_assert(!"EXIT");
    }
    s->radio_task = enmTaskTxEn;
    nrf51_radio_timer_start(s, true);
}

static void nrf51_radio_task_rxen(nrf51_radio_state *s)
{
    RDP("%s", "Trigger RXEN");
    s->ActiveConf.uFrequency = s->REG.FREQUENCY & 0x7f;
    //TODO: what is the decision point for MODE?
    s->ActiveConf.uMode = s->REG.MODE;
    g_assert(s->radio_state == enmStateDisabled);
    if (s->radio_task != enmTaskNone)
    {
        RDP("error: a task is already running: %s", task_to_str(s->radio_task));
        g_assert(!"EXIT");
    }
    s->radio_task = enmTaskRxEn;
    nrf51_radio_timer_start(s, true);
}

static void nrf51_radio_save_conf(nrf51_radio_state *s)
{
    int i,j;
    //Get address prefix (AP0..AP3)
    for (i = 0; i < 4; i++)
        s->ActiveConf.AP[i] = READ_MSB_BYTE(s->REG.PREFIX0, i);

    //Get address prefix (AP4..AP7)
    for (i = 0, j = 4; i < 4; i++, j++)
        s->ActiveConf.AP[j] = READ_MSB_BYTE(s->REG.PREFIX1, i);

    RDP("AP: %02x:%02x:%02x:%02x %02x:%02x:%02x:%02x",
        s->ActiveConf.AP[0], s->ActiveConf.AP[1],
        s->ActiveConf.AP[2], s->ActiveConf.AP[3],
        s->ActiveConf.AP[4], s->ActiveConf.AP[5],
        s->ActiveConf.AP[6], s->ActiveConf.AP[7]);

    //Save base addresses
    s->ActiveConf.BASE[0] = s->REG.BASE0;
    s->ActiveConf.BASE[1] = s->REG.BASE1;
    RDP("BASE0: 0x%x, BASE1: 0x%x", s->REG.BASE0, s->REG.BASE1);

    //Save address select
    s->ActiveConf.uTxAddrSelect = s->REG.TXADDRESS;
    RDP_DUMP(s->ActiveConf.uTxAddrSelect);

    //Save RX address enable bits.
    for (int i = 0; i < 8; i++)
    {
        //Extract ADDRx enable bits from register.
        s->ActiveConf.RXADDRESSES_ADDR[i] = GET_BIT(s->REG.RXADDRESSES, i);
//        RDP_DUMP(s->ActiveConf.RXADDRESSES_ADDR[i]);
    }

    //Save PCNF0 values
    s->ActiveConf.PCNF0.LFLEN = s->REG.PCNF0 & 0xf;
    s->ActiveConf.PCNF0.S0LEN = s->REG.PCNF0 >> 8 & 0x1;
    s->ActiveConf.PCNF0.S1LEN = s->REG.PCNF0 >> 16 & 0xf;

    RDP("PCNF0 {LFLEN: %u, S0LEN: %u, S1LEN: %u}",
        s->ActiveConf.PCNF0.LFLEN,
        s->ActiveConf.PCNF0.S0LEN,
        s->ActiveConf.PCNF0.S1LEN);

    //Save PCNF1 values
    s->ActiveConf.PCNF1.STATLEN = s->REG.PCNF1 & 0xff;
    s->ActiveConf.PCNF1.BALEN = (s->REG.PCNF1 >> 16) & 0x7;
    s->ActiveConf.PCNF1.ENDIAN = GET_BIT(s->REG.PCNF1, 24); //TODO: do conversion in packet later.

    RDP_DUMP(s->ActiveConf.PCNF1.STATLEN);
    RDP_DUMP(s->ActiveConf.PCNF1.BALEN);
    RDP_DUMP(s->ActiveConf.PCNF1.ENDIAN);
    //This value is active as soon as it is written to PCNF1;
    RDP_DUMP(s->ActiveConf.PCNF1.MAXLEN);

    //Save CRCCNF, CRCINIT, CRCPOLY
    s->ActiveConf.CRCCNF.uLen = s->REG.CRCCNF & 0x3;
    s->ActiveConf.CRCCNF.bSkipAddr = GET_BIT(s->REG.CRCCNF, 8);
    s->ActiveConf.CRCINIT = s->REG.CRCINIT;
    s->ActiveConf.CRCPOLY = s->REG.CRCPOLY;

    RDP("CRCCNF {LEN: %u, SKIPADDR: %u}",
        s->ActiveConf.CRCCNF.uLen,
        s->ActiveConf.CRCCNF.bSkipAddr);
    RDP("CRCINIT: 0x%x, CRCPOLY: 0x%x",
        s->ActiveConf.CRCINIT,
        s->ActiveConf.CRCPOLY);

    s->ActiveConf.PACKETPTR = s->REG.PACKETPTR;
}

static void nrf51_radio_timer_reload(nrf51_radio_state * s, int ns)
{
    if (!s->qtimer)
    {
        s->qtimer = timer_new_ns(QEMU_CLOCK_VIRTUAL, nrf51_radio_timer, s);
    }

    const int64_t expire = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
            ((ns > 0) ? ns : (NANOSECONDS_PER_SECOND / 8192 * 1000));
    timer_mod_ns(s->qtimer, expire);
}

void nrf51_complete_irq_17(void * opaque)
{
    nrf51_radio_state * const s = opaque;
    RDP("%s","IRQ 17 Complete");
    RAD_LOCK();
    s->irq17_pending = false;
    if (s->timer_running)
    {
        nrf51_radio_timer_reload(s, 1);
    }
    RAD_UNLOCK();
}

static void nrf51_radio_timer_start(nrf51_radio_state *s, bool run_first)
{
    if (!s->REG.POWER)
        return;

    if (s->timer_running)
    {
        return;
    }

    if (run_first)
    {
        s->timer_running = true; //TODO: added for testing, avoids assert.
        nrf51_radio_event_loop(s);
    }

    s->timer_running = true;
    nrf51_radio_timer_reload(s, 0);

    RDP("%s", "timer running");
}

static void nrf51_radio_timer_stop(nrf51_radio_state * s)
{
    if (!s->timer_running)
        RDP("warning: %s", "timer was not running");

    s->timer_running = false;

    RDP("%s", "timer stopped");
}

static bool fire_event_disabled(nrf51_radio_state *s)
{
    bool kill_timer = true;
    g_assert(s->radio_task == enmTaskNone);
    switch (s->radio_state)
    {
        case enmStateDisabled:
            g_assert(!(SHORT_DISABLED_RXEN(s) && SHORT_DISABLED_TXEN(s)));
            if (SHORT_DISABLED_RXEN(s))
            {
                RDP("%s (%u)", "set (S) DISABLED->RXEN", s->next_on_disabled);
                g_assert(s->next_on_disabled == enmTaskNone); //guest error
                s->next_on_disabled = enmTaskRxEn;
                kill_timer = false;
            }
            else if (SHORT_DISABLED_TXEN(s))
            {
                RDP("%s (%u)", "set (S) DISABLED->TXEN", s->next_on_disabled);
                g_assert(s->next_on_disabled == enmTaskNone); //guest error
                s->next_on_disabled = enmTaskTxEn;
                kill_timer = false;
            }
            else
            {
                //DISABLED fired from DISABLED state.
                //Cancel if there is a pending RxEn/TxEn task.
                s->next_on_disabled = enmTaskNone;
            }
            
            break;
        default:
            PRINT_INCORRECT_STATE();
            g_assert(!"incorrect state in fire event disabled");
            break;
    }

    s->REG.DISABLED = 1;
    s->irq17_pending = true;
    RDP("%s", "event DISABLED");
    nrf51_radio_pulse_event(s, RADIO_INTEN_DISABLED);

    return !kill_timer;
}

static const char * task_to_str(radio_task_t task)
{
    switch(task)
    {
        case enmTaskNone: return "NONE";
        case enmTaskTxEn: return "TXEN";
        case enmTaskRxEn: return "RXEN";
        case enmTaskStart: return "START";
        case enmTaskStop: return "STOP";
        case enmTaskDisable: return "DISABLE";
        default:
            return "UNK";
    }
}

static const char * state_to_str(NRF51_RADIO_STATE state)
{
    switch (state)
    {
        case enmStateDisabled: return "DISABLED";
        case enmStateRxRu: return "RXRU";
        case enmStateRxIdle: return "RXIDLE";
        case enmStateRx: return "RX";
        case enmStateTxRu: return "TXRU";
        case enmStateTxIdle: return "TXIDLE";
        case enmStateTx: return "TX";
        case enmStateRxDisable: return "RXDISABLE";
        case enmStateTxDisable: return "TXDISABLE";
        default:
            return "UNK";
    }
}

static int nrf51_radio_udp_send(nrf51_radio_state * s, nrf51_air_packet_buff * pkt)
{
    int udp_sz, sent;
    udp_sz = udp_read_len(&pkt->proto_hdr) + PROTO_HDR_SZ;
    sent = nrf51_udp_send((uint8_t*)pkt, udp_sz);
    if (sent == -1)
    {
        RDP("%s", "unable to send udp packet");
        return -1;
    }

    return sent;
}

void nrf51_radio_udp_init(nrf51_radio_state *s)
{
    qemu_mutex_init(&mtx_console);
    qemu_mutex_init(&mtx_radio);
}

static void nrf51_radio_pulse_event(const nrf51_radio_state * s, uint32_t mask)
{
    if (s->REG.INTEN & mask)
        qemu_irq_pulse(s->irq);
}

void nrf51_radio_do_rx(void * opaque, uint8_t * data, uint_fast16_t len)
{
    nrf51_radio_state * s = opaque; //TODO: null check?
    nrf51_air_packet * pkt = (nrf51_air_packet*) data;

    RDP("%s", "data received");

    if ( len < (int) (PROTO_HDR_SZ + AIR_PACKET_HDR_SZ) )
    {
        RDP("%s", "header not received");
        //No data received.
        return;// false;
    }

    if (pkt->type != enmAirTypeData)
    {
        RDP("drop, unexpected packet type: %u", pkt->type);
        return;// false;
    }

    RAD_LOCK();

    const int packet_len = udp_read_len(&pkt->proto_hdr) - AIR_PACKET_HDR_SZ;

    if ( s->radio_task != enmTaskNone || s->radio_state != enmStateRx )
    {
        //Drop this packet. We don't expect any radio data.
        RDP("%s", "Not expecting radio packet, drop");
        goto fail;
    }

    RDP("data received pay len: %u, packet len: %u", packet_len, len);
    hexdump("RX payload", pkt->data_start, packet_len);

    //FIXME: Calculate target data space.
    //FIXME: Truncate packet based on MAXLEN, SLEN, etc. params.
    if (NRF51_OUT_OF_RAM(s->ActiveConf.PACKETPTR, 4 /*FIXME: calculate me*/))
    {
        RDP("packetptr: 0x%x", s->ActiveConf.PACKETPTR);
        nrf51_trigger_hardfault();
        goto fail;
    }

    address_space_write(&address_space_memory, s->ActiveConf.PACKETPTR /*+ 2*/, MEMTXATTRS_UNSPECIFIED,
                        pkt->data_start, packet_len);

    s->REG.PAYLOAD = 1;
    RDP("%s", "event PAYLOAD");
    nrf51_radio_pulse_event(s, RADIO_INTEN_PAYLOAD);
    //FIXME: Do CRC operation. and IRQ?
    s->REG.CRCSTATUS = 1;

    s->REG.END = 1;
    RDP("%s", "event END");
    nrf51_radio_pulse_event(s, RADIO_INTEN_END);

    if (SHORT_END_DISABLE(s) && SHORT_END_START(s))
    {
        qemu_log_mask(LOG_GUEST_ERROR,
            "[RADIO] Rx: END_DISABLE & END_START are enabled at the same time.");
        RDP("guest error: %s", "disable timer");
        nrf51_radio_timer_stop(s);
        goto fail;
    }

    if (SHORT_END_DISABLE(s))
    {
        s->radio_task = enmTaskDisable;
    }
    else if (SHORT_END_START(s))
    {
        s->radio_task = enmTaskStart;
    }
    else
    {
        s->radio_state = enmStateRxIdle;
    }

    //Run the state machine. For immediate state transition.
    //Keep the timer running.
    nrf51_radio_timer_start(s, true);

    //Fall-through works but this is safer for possible future modifications
    RAD_UNLOCK();
    return;

fail:
    RAD_UNLOCK();
    //return true;
}

static void nrf51_radio_do_tx(nrf51_radio_state * s)
{
    unsigned int uHdrSize = 0;
    uint8_t achHdr[3];
    unsigned int uLength;
    unsigned int uPayLen = 0;
    unsigned int uRawSize;

    g_assert(s->radio_task == enmTaskNone);

    RDP("sending data pointed at 0x%x", s->ActiveConf.PACKETPTR);

    //This is either 1 byte or none.
    if(s->ActiveConf.PCNF0.S0LEN)
        uHdrSize++;

    //If LFLEN exists and its size is smaller than 8 bits
    //it will occupy 1 byte in memory.
    if (s->ActiveConf.PCNF0.LFLEN)
        uHdrSize++;

    //Same as LFLEN
    if(s->ActiveConf.PCNF0.S1LEN)
        uHdrSize++;

    if (uHdrSize > 0)
    {
        if (NRF51_OUT_OF_RAM(s->ActiveConf.PACKETPTR, uHdrSize))
        {
            nrf51_trigger_hardfault();
            return;
        }

        address_space_read(&address_space_memory, s->ActiveConf.PACKETPTR,
                           MEMTXATTRS_UNSPECIFIED, achHdr, (int) uHdrSize);
        if (s->ActiveConf.PCNF0.LFLEN)
        {
            if(s->ActiveConf.PCNF0.S0LEN)
                uLength = achHdr[1]; //Skip S0 and read length field
            else
                uLength = achHdr[0];
            //Do bit mask for configured number of bits.
            uLength &= (1 << s->ActiveConf.PCNF0.LFLEN) - 1;
            uPayLen += uLength;
        }
    }

    //Static length = number of bytes to transmit AFTER payload.
    uPayLen += s->ActiveConf.PCNF1.STATLEN;
    uRawSize = uHdrSize + uPayLen;

    //Raw packet size cannot exceed 254 bytes
    //Ref. manual, 17.1.3 Maximum packet length
    if (uRawSize > 254)
        uRawSize = 254;

    RDP("raw size: %d", uPayLen);
    RDP("pkt header size: %u", uHdrSize);
    RDP("payload size: %d", uPayLen);

    if (uRawSize == 254)
    {
        //It might be required to adjust maximum payload size.
        uPayLen = uRawSize - uHdrSize;
    }

    //TODO: Not clear if MAXLEN limits the payload size or total packet size. Assuming payload size.
    if (uPayLen > s->ActiveConf.PCNF1.MAXLEN)
        uPayLen = s->ActiveConf.PCNF1.MAXLEN;

    RDP("adjusted payload size: %d", uPayLen);

    if (uPayLen)
    {
        uint8_t payload[uPayLen];
        int i;
        address_space_read(&address_space_memory, s->ActiveConf.PACKETPTR /*+ uHdrSize*/,
                           MEMTXATTRS_UNSPECIFIED, payload, (int) uPayLen + uHdrSize);
        RDP("%s", "payload data: ");
        for (i = 0; i < uPayLen; i++)
        {
            printf("%02x ", payload[i]);
        }
        printf("\n");

        //build packet for udp layer
        air_packet.type = enmAirTypeData;
        air_packet.mode = s->ActiveConf.uMode;
        nrf51_udp_fill_hdr(&air_packet.proto_hdr, PROTO_RADIO, AIR_PACKET_HDR_SZ + uPayLen);
        assert(uPayLen <= PBUF_SZ);
        memcpy(air_packet.buffer, payload , uPayLen);

        nrf51_radio_udp_send(s, &air_packet);
    }

    s->REG.PAYLOAD = 1;
    RDP("%s", "event PAYLOAD");
    nrf51_radio_pulse_event(s, RADIO_INTEN_PAYLOAD);

    //FIXME: Do CRC operation.
    s->REG.END = 1;
    RDP("%s", "event END");
    nrf51_radio_pulse_event(s, RADIO_INTEN_END);

    if (SHORT_END_DISABLE(s) && SHORT_END_START(s))
    {
        qemu_log_mask(LOG_GUEST_ERROR,
            "[RADIO] Tx: END_DISABLE & END_START are enabled at the same time.");
        RDP("guest error: %s", "disable timer");
        nrf51_radio_timer_stop(s);
        return;
    }

    //Set new state, following shorts may override it.
    s->radio_state = enmStateTxIdle;

    if (SHORT_END_DISABLE(s))
    {
        //This is disable task.
        s->radio_state = enmStateTxDisable;
        s->radio_task = enmTaskNone;
    }
    else if (SHORT_END_START(s))
    {
        s->radio_state = enmStateTxIdle;
        s->radio_task = enmTaskStart;
    }

    g_assert(s->timer_running);

    nrf51_radio_event_loop(s);
}

static void state_disabled(nrf51_radio_state *s)
{
    switch (s->radio_task)
    {
        case enmTaskTxEn:
            s->radio_state = enmStateTxRu;
            s->radio_task = enmTaskNone;
            nrf51_radio_event_loop(s);
            break;

        case enmTaskRxEn:
            s->radio_state = enmStateRxRu;
            s->radio_task = enmTaskNone;
            nrf51_radio_event_loop(s);
            break;

        case enmTaskDisable:
            s->radio_task = enmTaskNone;
            s->next_on_disabled = enmTaskNone; //DISABLE fired while in DISABLED state, so cancel next task.
            fire_event_disabled(s); //TODO: disabled event from disabled state? Is it correct?
            break;

        case enmTaskNone:
            //Timer not killed, should kill if no interrupt?
            break;

        default:
            PRINT_INCORRECT_STATE();
            nrf51_radio_timer_stop(s);
            break;
    }
}

static void state_rxtxru(nrf51_radio_state *s)
{
    switch (s->radio_task)
    {
       case enmTaskNone:
            RDP("event (%s) READY", (s->radio_state == enmStateRxRu)  ? "RX" : "TX");
            s->radio_state = (s->radio_state == enmStateRxRu)  ? enmStateRxIdle : enmStateTxIdle;
            s->REG.READY = 1;
            nrf51_radio_event_ready_ppi();
            nrf51_radio_pulse_event(s, RADIO_INTEN_READY);
            if (SHORT_READY_START(s))
            {
                s->radio_task = enmTaskStart;
                nrf51_radio_event_loop(s);
            }
            break;

        case enmTaskDisable:
            s->radio_task = enmTaskNone; //Finish task
            s->radio_state = enmStateRxDisable; //New state
            nrf51_radio_event_loop(s);
            break;

        default:
            PRINT_INCORRECT_STATE();
            nrf51_radio_timer_stop(s);
            break;
    }
}

static void state_rxtxidle(nrf51_radio_state *s)
{
    g_assert(s->radio_state == enmStateRxIdle || s->radio_state == enmStateTxIdle);

    switch (s->radio_task)
    {
        case enmTaskNone:
        {
            break;
        }

        case enmTaskStart:
        {
            nrf51_radio_save_conf(s);
            s->radio_task = enmTaskNone;

            if (s->radio_state == enmStateRxIdle)
            {
                s->radio_state = enmStateRx;
            }
            else //TxIdle
            {
                s->radio_state = enmStateTx;
            }
            nrf51_radio_event_loop(s);
            break;
        }

        case enmTaskDisable:
        {
            s->radio_task = enmTaskNone;
            s->radio_state = (s->radio_state == enmStateRxIdle) ?
                    enmStateRxDisable : enmStateTxDisable;
            nrf51_radio_event_loop(s);
            break;
        }

        default:
            PRINT_INCORRECT_STATE();
            nrf51_radio_timer_stop(s);
            break;
    }

    //No code should come here. (tail-rec call)
}

static void state_rx(nrf51_radio_state *s)
{
    switch(s->radio_task)
    {
        case enmTaskNone:
        {
            //Stay in RX state until packet is received
            break;
        }

        case enmTaskDisable:
        {
            s->radio_task = enmTaskNone;
            s->radio_state = enmStateRxDisable;
            nrf51_radio_event_loop(s);
            break;
        }

        default:
            PRINT_INCORRECT_STATE();
            nrf51_radio_timer_stop(s);
            break;
    }

    //tail-rec, nothing comes here
}

static void state_tx(nrf51_radio_state *s)
{
    switch(s->radio_task)
    {
        case enmTaskNone:
        {
            //state change handled by do_tx
            nrf51_radio_do_tx(s);
            //s->radio_state = enmStateTxIdle;
            break;
        }

        default:
            PRINT_INCORRECT_STATE();
            nrf51_radio_timer_stop(s);
            break;
    }
}

static void state_rxtxdisable(nrf51_radio_state *s)
{
    switch (s->radio_task)
    {
        case enmTaskNone:
        {
            s->radio_state = enmStateDisabled;
            fire_event_disabled(s);
            break;
        }

        default:
            PRINT_INCORRECT_STATE();
            nrf51_radio_timer_stop(s);
            break;
    }
}

static void check_pending_task(nrf51_radio_state * const s)
{

    if (s->radio_task == enmTaskDisable)
    {
        s->next_on_disabled = enmTaskNone;
        return;
    }

    //TODO: check if two shorts are enabled at the same time.
    if (s->next_on_disabled == enmTaskTxEn /*&& SHORT_DISABLED_TXEN(s)*/)
    {
        RDP("%s", "run (S) DISABLED->TXEN");
        s->next_on_disabled = enmTaskNone;
        nrf51_radio_task_txen(s);
    }
    else if (s->next_on_disabled == enmTaskRxEn /*&&  SHORT_DISABLED_RXEN(s)*/)
    {
        RDP("%s", "run (S) DISABLED->RXEN");
        s->next_on_disabled = enmTaskNone;
        nrf51_radio_task_rxen(s);
    }
    else
    {
        RDP("Next task was not run, shorts changed: 0x%x", s->REG.SHORTS);
    }

}

void nrf51_radio_timer(void *opaque)
{
    nrf51_radio_state * const s = opaque;
    RAD_LOCK();
    if(!s->timer_running)
    {
        RAD_UNLOCK();
        return;
    }

    if (s->irq17_pending)
    {
        nrf51_radio_timer_reload(s, 0);
        RAD_UNLOCK();
        return;
    }

    if (s->next_on_disabled != enmTaskNone)
    {
        check_pending_task(s);
    }

    nrf51_radio_event_loop(opaque);
    nrf51_radio_timer_reload(s, 0);

    RAD_UNLOCK();
}

static void nrf51_radio_event_loop(void *opaque)
{
    nrf51_radio_state * const s = opaque;

    if (!s->REG.POWER)
    {
        RDP("%s", "powered off, stop timer.");
        if (s->timer_running)
            nrf51_radio_timer_stop(s);
        s->radio_state = enmStateDisabled;
        s->radio_task = enmTaskNone;
        return;
    }

    {
        static int last;
        int now = (s->radio_state << 16) | s->radio_task;
        if (now != last)
        {
            RDP("current state: %s, task: %s", state_to_str(s->radio_state), task_to_str(s->radio_task));
            last = now;
        }
    }

    /* Implemented according to ref. manual v3.0 Figure 22: Radio states */
    /* RADIO does not prevent a task being triggered from wrong state (ref. manual 17.1.8 Radio States, p. 84) */

    const state_func_t fn = state_fns[s->radio_state];

    if (fn)
    {
        fn(s);
        return;
    }
    else
    {
            RDP("unknown radio state: %u (task: %u)",
                s->radio_state, s->radio_task);
            nrf51_radio_timer_stop(s);
    }

    //No code should come here. (tail-rec call)
}
