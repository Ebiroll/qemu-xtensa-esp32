#pragma once

#define PRO_CPU_NUM (0)


#define SOC_EXTRAM_DATA_LOW 0x3F500000
#define SOC_EXTRAM_DATA_HIGH 0x3FF80000

#define SOC_MAX_CONTIGUOUS_RAM_SIZE (SOC_EXTRAM_DATA_HIGH - SOC_EXTRAM_DATA_LOW) ///< Largest span of contiguous memory (DRAM or IRAM) in the address space

#define DR_REG_SYSTEM_BASE                      0x3f4c0000
#define DR_REG_SENSITIVE_BASE                   0x3f4c1000
#define DR_REG_INTERRUPT_BASE                   0x3f4c2000
#define DR_REG_DMA_COPY_BASE                    0x3f4c3000
#define DR_REG_EXTMEM_BASE                      0x61800000
#define DR_REG_MMU_TABLE                        0x61801000
#define DR_REG_ITAG_TABLE                       0x61802000
#define DR_REG_DTAG_TABLE                       0x61803000
#define DR_REG_AES_BASE                         0x6003a000
#define DR_REG_SHA_BASE                         0x6003b000
#define DR_REG_RSA_BASE                         0x6003c000
#define DR_REG_HMAC_BASE                        0x6003e000
#define DR_REG_DIGITAL_SIGNATURE_BASE           0x6003d000
#define DR_REG_CRYPTO_DMA_BASE                  0x6003f000
#define DR_REG_ASSIST_DEBUG_BASE                0x3f4ce000
#define DR_REG_DEDICATED_GPIO_BASE              0x3f4cf000
#define DR_REG_INTRUSION_BASE                   0x3f4d0000
#define DR_REG_DPORT_END                        0x3f4d3FFC
#define DR_REG_UART_BASE                        0x3f400000
#define DR_REG_SPI1_BASE                        0x3f402000
#define DR_REG_SPI0_BASE                        0x3f403000
#define DR_REG_GPIO_BASE                        0x3f404000
#define DR_REG_GPIO_SD_BASE                     0x3f404f00
#define DR_REG_FE2_BASE                         0x3f405000
#define DR_REG_FE_BASE                          0x3f406000
#define DR_REG_FRC_TIMER_BASE                   0x3f407000
#define DR_REG_RTCCNTL_BASE                     0x3f408000
#define DR_REG_RTCIO_BASE                       0x3f408400
#define DR_REG_SENS_BASE                        0x3f408800
#define DR_REG_RTC_I2C_BASE                     0x3f408C00
#define DR_REG_IO_MUX_BASE                      0x3f409000
#define DR_REG_HINF_BASE                        0x3f40B000
#define DR_REG_I2S_BASE                         0x3f40F000
#define DR_REG_UART1_BASE                       0x3f410000
#define DR_REG_I2C_EXT_BASE                     0x3f413000
#define DR_REG_UHCI0_BASE                       0x3f414000
#define DR_REG_SLCHOST_BASE                     0x3f415000
#define DR_REG_RMT_BASE                         0x3f416000
#define DR_REG_PCNT_BASE                        0x3f417000
#define DR_REG_SLC_BASE                         0x3f418000
#define DR_REG_LEDC_BASE                        0x3f419000
#define DR_REG_MCP_BASE                         0x3f4c3000
#define DR_REG_EFUSE_BASE                       0x3f41A000
#define DR_REG_NRX_BASE                         0x3f41CC00
#define DR_REG_BB_BASE                          0x3f41D000
#define DR_REG_TIMERGROUP0_BASE                 0x3f41F000
#define DR_REG_TIMERGROUP1_BASE                 0x3f420000
#define DR_REG_RTC_SLOWMEM_BASE                 0x3f421000
#define DR_REG_SYSTIMER_BASE                    0x3f423000
#define DR_REG_SPI2_BASE                        0x3f424000
#define DR_REG_SPI3_BASE                        0x3f425000
#define DR_REG_SYSCON_BASE                      0x3f426000
#define DR_REG_APB_CTRL_BASE                    0x3f426000    /* Old name for SYSCON, to be removed */
#define DR_REG_I2C1_EXT_BASE                    0x3f427000
#define DR_REG_SPI4_BASE                        0x3f437000
#define DR_REG_USB_WRAP_BASE                    0x3f439000
#define DR_REG_APB_SARADC_BASE                  0x3f440000
#define DR_REG_USB_BASE                         0x60080000

#define REG_UHCI_BASE(i)         (DR_REG_UHCI0_BASE)
#define REG_UART_BASE( i )  (DR_REG_UART_BASE + (i) * 0x10000 )
#define REG_UART_AHB_BASE(i)  (0x60000000 + (i) * 0x10000 )
#define UART_FIFO_AHB_REG(i)  (REG_UART_AHB_BASE(i) + 0x0)
#define REG_I2S_BASE( i ) (DR_REG_I2S_BASE)
#define REG_TIMG_BASE(i)              (DR_REG_TIMERGROUP0_BASE + (i)*0x1000)
#define REG_SPI_MEM_BASE(i)     (DR_REG_SPI0_BASE - (i) * 0x1000)
#define REG_I2C_BASE(i)    (DR_REG_I2C_EXT_BASE + (i) * 0x14000 )




#define APB_REG_BASE                            0x60000000


//interrupt cpu using table, Please see the core-isa.h
/*************************************************************************************************************
 *      Intr num                Level           Type                    PRO CPU usage           APP CPU uasge
 *      0                       1               extern level            WMAC                    Reserved
 *      1                       1               extern level            BT/BLE Host HCI DMA     BT/BLE Host HCI DMA
 *      2                       1               extern level
 *      3                       1               extern level
 *      4                       1               extern level            WBB
 *      5                       1               extern level            BT/BLE Controller       BT/BLE Controller
 *      6                       1               timer                   FreeRTOS Tick(L1)       FreeRTOS Tick(L1)
 *      7                       1               software                BT/BLE VHCI             BT/BLE VHCI
 *      8                       1               extern level            BT/BLE BB(RX/TX)        BT/BLE BB(RX/TX)
 *      9                       1               extern level
 *      10                      1               extern edge
 *      11                      3               profiling
 *      12                      1               extern level
 *      13                      1               extern level
 *      14                      7               nmi                     Reserved                Reserved
 *      15                      3               timer                   FreeRTOS Tick(L3)       FreeRTOS Tick(L3)
 *      16                      5               timer
 *      17                      1               extern level
 *      18                      1               extern level
 *      19                      2               extern level
 *      20                      2               extern level
 *      21                      2               extern level
 *      22                      3               extern edge
 *      23                      3               extern level
 *      24                      4               extern level            TG1_WDT
 *      25                      4               extern level            CACHEERR
 *      26                      5               extern level
 *      27                      3               extern level            Reserved                Reserved
 *      28                      4               extern edge             DPORT ACCESS            DPORT ACCESS
 *      29                      3               software                Reserved                Reserved
 *      30                      4               extern edge             Reserved                Reserved
 *      31                      5               extern level
 *************************************************************************************************************
 */

// TODO!!! FIX OLAS
#define ETS_WIFI_MAC_INTR_SOURCE                0/**< interrupt of WiFi MAC, level*/
#define ETS_WIFI_MAC_NMI_SOURCE                 1/**< interrupt of WiFi MAC, NMI, use if MAC have bug to fix in NMI*/
#define ETS_WIFI_BB_INTR_SOURCE                 2/**< interrupt of WiFi BB, level, we can do some calibartion*/
#define ETS_BT_MAC_INTR_SOURCE                  3/**< will be cancelled*/
#define ETS_BT_BB_INTR_SOURCE                   4/**< interrupt of BT BB, level*/
#define ETS_BT_BB_NMI_SOURCE                    5/**< interrupt of BT BB, NMI, use if BB have bug to fix in NMI*/
#define ETS_RWBT_INTR_SOURCE                    6/**< interrupt of RWBT, level*/
#define ETS_RWBLE_INTR_SOURCE                   7/**< interrupt of RWBLE, level*/
#define ETS_RWBT_NMI_SOURCE                     8/**< interrupt of RWBT, NMI, use if RWBT have bug to fix in NMI*/
#define ETS_RWBLE_NMI_SOURCE                    9/**< interrupt of RWBLE, NMI, use if RWBT have bug to fix in NMI*/
#define ETS_SLC0_INTR_SOURCE                    10/**< interrupt of SLC0, level*/
#define ETS_SLC1_INTR_SOURCE                    11/**< interrupt of SLC1, level*/
#define ETS_UHCI0_INTR_SOURCE                   12/**< interrupt of UHCI0, level*/
#define ETS_UHCI1_INTR_SOURCE                   13/**< interrupt of UHCI1, level*/
#define ETS_TG0_T0_LEVEL_INTR_SOURCE            14/**< interrupt of TIMER_GROUP0, TIMER0, level, we would like use EDGE for timer if permission*/
#define ETS_TG0_T1_LEVEL_INTR_SOURCE            15/**< interrupt of TIMER_GROUP0, TIMER1, level, we would like use EDGE for timer if permission*/
#define ETS_TG0_WDT_LEVEL_INTR_SOURCE           16/**< interrupt of TIMER_GROUP0, WATCHDOG, level*/
#define ETS_TG0_LACT_LEVEL_INTR_SOURCE          17/**< interrupt of TIMER_GROUP0, LACT, level*/
#define ETS_TG1_T0_LEVEL_INTR_SOURCE            18/**< interrupt of TIMER_GROUP1, TIMER0, level, we would like use EDGE for timer if permission*/
#define ETS_TG1_T1_LEVEL_INTR_SOURCE            19/**< interrupt of TIMER_GROUP1, TIMER1, level, we would like use EDGE for timer if permission*/
#define ETS_TG1_WDT_LEVEL_INTR_SOURCE           20/**< interrupt of TIMER_GROUP1, WATCHDOG, level*/
#define ETS_TG1_LACT_LEVEL_INTR_SOURCE          21/**< interrupt of TIMER_GROUP1, LACT, level*/
#define ETS_GPIO_INTR_SOURCE                    22/**< interrupt of GPIO, level*/
#define ETS_GPIO_NMI_SOURCE                     23/**< interrupt of GPIO, NMI*/
#define ETS_FROM_CPU_INTR0_SOURCE               24/**< interrupt0 generated from a CPU, level*/ /* Used for FreeRTOS */
#define ETS_FROM_CPU_INTR1_SOURCE               25/**< interrupt1 generated from a CPU, level*/ /* Used for FreeRTOS */
#define ETS_FROM_CPU_INTR2_SOURCE               26/**< interrupt2 generated from a CPU, level*/ /* Used for DPORT Access */
#define ETS_FROM_CPU_INTR3_SOURCE               27/**< interrupt3 generated from a CPU, level*/ /* Used for DPORT Access */
#define ETS_SPI0_INTR_SOURCE                    28/**< interrupt of SPI0, level, SPI0 is for Cache Access, do not use this*/
#define ETS_SPI1_INTR_SOURCE                    29/**< interrupt of SPI1, level, SPI1 is for flash read/write, do not use this*/
#define ETS_SPI2_INTR_SOURCE                    30/**< interrupt of SPI2, level*/
#define ETS_SPI3_INTR_SOURCE                    31/**< interrupt of SPI3, level*/
#define ETS_I2S0_INTR_SOURCE                    32/**< interrupt of I2S0, level*/
#define ETS_I2S1_INTR_SOURCE                    33/**< interrupt of I2S1, level*/
#define ETS_UART0_INTR_SOURCE                   34/**< interrupt of UART0, level*/
#define ETS_UART1_INTR_SOURCE                   35/**< interrupt of UART1, level*/
#define ETS_UART2_INTR_SOURCE                   36/**< interrupt of UART2, level*/
#define ETS_SDIO_HOST_INTR_SOURCE               37/**< interrupt of SD/SDIO/MMC HOST, level*/
#define ETS_ETH_MAC_INTR_SOURCE                 38/**< interrupt of ethernet mac, level*/
#define ETS_PWM0_INTR_SOURCE                    39/**< interrupt of PWM0, level, Reserved*/
#define ETS_PWM1_INTR_SOURCE                    40/**< interrupt of PWM1, level, Reserved*/
#define ETS_PWM2_INTR_SOURCE                    41/**< interrupt of PWM2, level*/
#define ETS_PWM3_INTR_SOURCE                    42/**< interruot of PWM3, level*/
#define ETS_LEDC_INTR_SOURCE                    43/**< interrupt of LED PWM, level*/
#define ETS_EFUSE_INTR_SOURCE                   44/**< interrupt of efuse, level, not likely to use*/
#define ETS_CAN_INTR_SOURCE                     45/**< interrupt of can, level*/
#define ETS_RTC_CORE_INTR_SOURCE                46/**< interrupt of rtc core, level, include rtc watchdog*/
#define ETS_RMT_INTR_SOURCE                     47/**< interrupt of remote controller, level*/
#define ETS_PCNT_INTR_SOURCE                    48/**< interrupt of pluse count, level*/
#define ETS_I2C_EXT0_INTR_SOURCE                49/**< interrupt of I2C controller1, level*/
#define ETS_I2C_EXT1_INTR_SOURCE                50/**< interrupt of I2C controller0, level*/
#define ETS_RSA_INTR_SOURCE                     51/**< interrupt of RSA accelerator, level*/
#define ETS_SPI1_DMA_INTR_SOURCE                52/**< interrupt of SPI1 DMA, SPI1 is for flash read/write, do not use this*/
#define ETS_SPI2_DMA_INTR_SOURCE                53/**< interrupt of SPI2 DMA, level*/
#define ETS_SPI3_DMA_INTR_SOURCE                54/**< interrupt of SPI3 DMA, level*/
#define ETS_WDT_INTR_SOURCE                     55/**< will be cancelled*/
#define ETS_TIMER1_INTR_SOURCE                  56/**< will be cancelled*/
#define ETS_TIMER2_INTR_SOURCE                  57/**< will be cancelled*/
#define ETS_TG0_T0_EDGE_INTR_SOURCE             58/**< interrupt of TIMER_GROUP0, TIMER0, EDGE*/
#define ETS_TG0_T1_EDGE_INTR_SOURCE             59/**< interrupt of TIMER_GROUP0, TIMER1, EDGE*/
#define ETS_TG0_WDT_EDGE_INTR_SOURCE            60/**< interrupt of TIMER_GROUP0, WATCH DOG, EDGE*/
#define ETS_TG0_LACT_EDGE_INTR_SOURCE           61/**< interrupt of TIMER_GROUP0, LACT, EDGE*/
#define ETS_TG1_T0_EDGE_INTR_SOURCE             62/**< interrupt of TIMER_GROUP1, TIMER0, EDGE*/
#define ETS_TG1_T1_EDGE_INTR_SOURCE             63/**< interrupt of TIMER_GROUP1, TIMER1, EDGE*/
#define ETS_TG1_WDT_EDGE_INTR_SOURCE            64/**< interrupt of TIMER_GROUP1, WATCHDOG, EDGE*/
#define ETS_TG1_LACT_EDGE_INTR_SOURCE           65/**< interrupt of TIMER_GROUP0, LACT, EDGE*/
#define ETS_MMU_IA_INTR_SOURCE                  66/**< interrupt of MMU Invalid Access, LEVEL*/
#define ETS_MPU_IA_INTR_SOURCE                  67/**< interrupt of MPU Invalid Access, LEVEL*/
#define ETS_CACHE_IA_INTR_SOURCE                68/**< interrupt of Cache Invalied Access, LEVEL*/


#define ESP32_DPORT_CROSSCORE_INT_COUNT     4
#define ESP32_INT_MATRIX_OUTPUTS    45
#define ESP32_INT_MATRIX_INPUTS     69
#define ESP32_CPU_COUNT             1
#define ESP32_UART_COUNT            2
#define ESP32_FRC_COUNT             2
#define ESP32_TIMG_COUNT            2
#define ESP32_SPI_COUNT             4
#define ESP32_RTC_CNTL_SCRATCH_REG_COUNT     8

