#include "nrf51_helper.h"
#include "hw/misc/unimp.h"
#include "nrf51.h"
#include "nrf51_radio.h"

#define EP_CASE(mod,ev) case mod##_BASE + O_##mod##_##ev: do{return #mod "_" #ev;}while(0)
#define EP_CASE_N(mod,n,ev) case mod##n##_BASE + O_##mod##_##ev: do{return #mod#n "_" #ev;}while(0)
const char * eptostr(uint32_t ep_addr)
{
    __thread static char ret[16];
    switch (ep_addr)
    {
        EP_CASE_N(TIMER, 0, START);
        EP_CASE_N(TIMER, 0, CLEAR);
        EP_CASE_N(TIMER, 0, COMPARE0);
        EP_CASE_N(TIMER, 0, COMPARE1);
        EP_CASE_N(TIMER, 0, COMPARE2);
        EP_CASE_N(TIMER, 0, COMPARE3);
        EP_CASE_N(TIMER, 0, CAPTURE0);
        EP_CASE_N(TIMER, 0, CAPTURE1);
        EP_CASE_N(TIMER, 0, CAPTURE2);
        EP_CASE_N(TIMER, 0, CAPTURE3);

        EP_CASE_N(TIMER, 1, START);
        EP_CASE_N(TIMER, 1, CLEAR);
        EP_CASE_N(TIMER, 1, COMPARE0);
        EP_CASE_N(TIMER, 1, COMPARE1);
        EP_CASE_N(TIMER, 1, COMPARE2);
        EP_CASE_N(TIMER, 1, COMPARE3);
        EP_CASE_N(TIMER, 1, CAPTURE0);
        EP_CASE_N(TIMER, 1, CAPTURE1);
        EP_CASE_N(TIMER, 1, CAPTURE2);
        EP_CASE_N(TIMER, 1, CAPTURE3);

        EP_CASE_N(RTC, 0, COMPARE0);
        EP_CASE_N(RTC, 0, COMPARE1);
        EP_CASE_N(RTC, 0, COMPARE2);
        EP_CASE_N(RTC, 0, COMPARE3);

        EP_CASE_N(RTC, 1, COMPARE0);
        EP_CASE_N(RTC, 1, COMPARE1);
        EP_CASE_N(RTC, 1, COMPARE2);
        EP_CASE_N(RTC, 1, COMPARE3);

        EP_CASE(RADIO, END);
        EP_CASE(RADIO, RXEN);
        EP_CASE(RADIO, TXEN);
        EP_CASE(RADIO, READY);
        EP_CASE(RADIO, DISABLE);
        EP_CASE(RADIO, ADDRESS);

        EP_CASE(CCM, KSGEN);
        EP_CASE(CCM, CRYPT);
        default:
            snprintf(ret, 16, "0x%x", ep_addr);
            return ret;
    }

    return NULL;
}

const char * regtostr(uint32_t reg)
{
    switch (reg)
    {
        //Tasks
        case 0x0: return "T_RADIO_TXEN";
        case 0x4: return "T_RADIO_RXEN";
        case 0x8: return "T_RADIO_START";
        case 0xC: return "T_RADIO_STOP";
        case 0x10: return "T_RADIO_DISABLE";
        case 0x14: return "T_RADIO_RSSISTART";
        case 0x18: return "T_RADIO_RSSISTOP";
        case 0x1C: return "T_RADIO_BCSTART";
        case 0x20: return "T_RADIO_BCSTOP";

        //Events
        case 0x100: return "O_RADIO_READY";
        case 0x104: return "O_RADIO_ADDRESS";
        case 0x108: return "O_RADIO_PAYLOAD";
        case 0x10C: return "O_RADIO_END";
        case 0x110: return "O_RADIO_DISABLED";
        case 0x114: return "O_RADIO_DEVMATCH";
        case 0x118: return "O_RADIO_DEVMISS";
        case 0x11C: return "O_RADIO_RSSIEND";
        case 0x128: return "O_RADIO_BCMATCH";

        //Registers
        case 0x200: return "O_RADIO_SHORTS";
        case 0x304: return "O_RADIO_INTENSET";
        case 0x308: return "O_RADIO_INTENCLR";
        case 0x400: return "O_RADIO_CRCSTATUS";
        case 0x408: return "O_RADIO_RXMATCH";
        case 0x40C: return "O_RADIO_RXCRC";
        case 0x410: return "O_RADIO_DAI";
        case 0x504: return "O_RADIO_PACKETPTR";
        case 0x508: return "O_RADIO_FREQUENCY";
        case 0x50C: return "O_RADIO_TXPOWER";
        case 0x510: return "O_RADIO_MODE";
        case 0x514: return "O_RADIO_PCNF0";
        case 0x518: return "O_RADIO_PCNF1";
        case 0x51C: return "O_RADIO_BASE0";
        case 0x520: return "O_RADIO_BASE1";
        case 0x524: return "O_RADIO_PREFIX0";
        case 0x528: return "O_RADIO_PREFIX1";
        case 0x52C: return "O_RADIO_TXADDRESS";
        case 0x530: return "O_RADIO_RXADDRESSES";
        case 0x534: return "O_RADIO_CRCCNF";
        case 0x538: return "O_RADIO_CRCPOLY";
        case 0x53C: return "O_RADIO_CRCINIT";
        case 0x540: return "O_RADIO_TEST";
        case 0x544: return "O_RADIO_TIFS";
        case 0x548: return "O_RADIO_RSSISAMPLE";
        case 0x550: return "O_RADIO_STATE";
        case 0x554: return "O_RADIO_DATAWHITEIV";
        case 0x560: return "O_RADIO_BCC";
        case 0x600: return "O_RADIO_DAB0";
        case 0x604: return "O_RADIO_DAB1";
        case 0x608: return "O_RADIO_DAB2";
        case 0x60C: return "O_RADIO_DAB3";
        case 0x610: return "O_RADIO_DAB4";
        case 0x614: return "O_RADIO_DAB5";
        case 0x618: return "O_RADIO_DAB6";
        case 0x61C: return "O_RADIO_DAB7";

        case 0x620: return "O_RADIO_DAP0";
        case 0x624: return "O_RADIO_DAP1";
        case 0x628: return "O_RADIO_DAP2";
        case 0x62C: return "O_RADIO_DAP3";
        case 0x630: return "O_RADIO_DAP4";
        case 0x634: return "O_RADIO_DAP5";
        case 0x648: return "O_RADIO_DAP6";
        case 0x63C: return "O_RADIO_DAP7";

        case 0x640: return "O_RADIO_DACNF";

        case 0x724: return "O_RADIO_OVERRIDE0";
        case 0x728: return "O_RADIO_OVERRIDE2";
        case 0x72C: return "O_RADIO_OVERRIDE3";
        case 0x730: return "O_RADIO_OVERRIDE4";
        case 0x734: return "O_RADIO_OVERRIDE5";

        case 0xFFC: return "O_RADIO_POWER";

        default: return "UNKNOWN";
    }
}
