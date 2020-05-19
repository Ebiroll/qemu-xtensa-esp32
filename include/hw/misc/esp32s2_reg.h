#pragma once

#define PRO_CPU_NUM (0)


#define SOC_EXTRAM_DATA_LOW 0x3F500000
#define SOC_EXTRAM_DATA_HIGH 0x3FF80000

#define SOC_MAX_CONTIGUOUS_RAM_SIZE (SOC_EXTRAM_DATA_HIGH - SOC_EXTRAM_DATA_LOW) ///< Largest span of contiguous memory (DRAM or IRAM) in the address space


/* 
PeriBUS1 0x3F4C1000

mission Control
Name Description Address Access
Control Registers
//IBUS permission control register 0.
PMS_PRO_IRAM0_0_REG  0x0010 R/W
PMS_PRO_DRAM0_0_REG DBUS permission control register 0. 0x0028 R/W
PMS_PRO_DPORT_0_REG PeriBus1 permission control register 0. 0x003C R/W
PMS_PRO_AHB_0_REG PeriBus2 permission control register 0. 0x005C R/W
PMS_PRO_TRACE_0_REG Trace memory permission control register 0. 0x0070 R/W
PMS_PRO_CACHE_0_REG Cache permission control register 0. 0x0078 R/W
PMS_DMA_APB_I_0_REG Internal DMA permission control register 0. 0x008C R/W
PMS_DMA_RX_I_0_REG RX Copy DMA permission control register 0. 0x009C R/W
PMS_DMA_TX_I_0_REG TX Copy DMA permission control register 0. 0x00AC R/W
PMS_CACHE_SOURCE_0_REG Cache access permission control register 0. 0x00C4 R/W
PMS_APB_PERIPHERAL_0_REG Peripheral access permission control register 0. 0x00CC R/W
PMS_OCCUPY_0_REG Occupy permission control register 0. 0x00D4 R/W
PMS_CACHE_TAG_ACCESS_0_REG Cache tag permission control register 0. 0x00E4 R/W
PMS_CACHE_MMU_ACCESS_0_REG Cache MMU permission control register 0. 0x00EC R/W
PMS_CLOCK_GATE_REG_REG Clock gate register of permission control. 0x0104 R/W
Configuration Registers
PMS_PRO_IRAM0_1_REG IBUS permission control register 1. 0x0014 R/W
PMS_PRO_IRAM0_2_REG IBUS permission control register 2. 0x0018 R/W
PMS_PRO_IRAM0_3_REG IBUS permission control register 3. 0x001C R/W
PMS_PRO_DRAM0_1_REG DBUS permission control register 1. 0x002C R/W
PMS_PRO_DRAM0_2_REG DBUS permission control register 2. 0x0030 R/W
PMS_PRO_DPORT_1_REG PeriBus1 permission control register 1. 0x0040 R/W
PMS_PRO_DPORT_2_REG PeriBus1 permission control register 2. 0x0044 R/W
PMS_PRO_DPORT_3_REG PeriBus1 permission control register 3. 0x0048 R/W
PMS_PRO_DPORT_4_REG PeriBus1 permission control register 4. 0x004C R/W
PMS_PRO_DPORT_5_REG PeriBus1 permission control register 5. 0x0050 R/W
PMS_PRO_AHB_1_REG PeriBus2 permission control register 1. 0x0060 R/W
PMS_PRO_AHB_2_REG PeriBus2 permission control register 2. 0x0064 R/W
PMS_PRO_TRACE_1_REG Trace memory permission control register 1. 0x0074 R/W
PMS_PRO_CACHE_1_REG Cache permission control register 1. 0x007C R/W
PMS_DMA_APB_I_1_REG Internal DMA permission control register 1. 0x0090 R/W
PMS_DMA_RX_I_1_REG RX Copy DMA permission control register 1. 0x00A0 R/W
PMS_DMA_TX_I_1_REG TX Copy DMA permission control register 1. 0x00B0 R/W
PMS_APB_PERIPHERAL_1_REG Peripheral access permission control register 1. 0x00D0 R/W
PMS_OCCUPY_1_REG Occupy permission control register 1. 0x00D8 R/W
PMS_OCCUPY_3_REG Occupy permission control register 3. 0x00E0 R/W
PMS_CACHE_TAG_ACCESS_1_REG Cache tag permission control register 1. 0x00E8 R/W
PMS_CACHE_MMU_ACCESS_1_REG Cache MMU permission control register 1. 0x00F0 R/W
Interrupt Registers
PMS_PRO_IRAM0_4_REG IBUS permission control register 4. 0x0020 varies
PMS_PRO_IRAM0_5_REG IBUS status register. 0x0024 RO
PMS_PRO_DRAM0_3_REG DBUS permission control register 3. 0x0034 varies
Espressif Systems 437
Submit Documentation Feedback
ESP32-S2 TRM (Preliminary V0.3)
23. Permission Control
Name Description Address Access
PMS_PRO_DRAM0_4_REG DBUS status register. 0x0038 RO
PMS_PRO_DPORT_6_REG PeriBus1 permission control register 6. 0x0054 varies
PMS_PRO_DPORT_7_REG PeriBus1 status register. 0x0058 RO
PMS_PRO_AHB_3_REG PeriBus2 permission control register 3. 0x0068 varies
PMS_PRO_AHB_4_REG PeriBus2 status register. 0x006C RO
PMS_PRO_CACHE_2_REG Cache permission control register 2. 0x0080 varies
PMS_PRO_CACHE_3_REG Icache status register. 0x0084 RO
PMS_PRO_CACHE_4_REG Dcache status register. 0x0088 RO
PMS_DMA_APB_I_2_REG Internal DMA permission control register 2. 0x0094 varies
PMS_DMA_APB_I_3_REG Internal DMA status register. 0x0098 RO
PMS_DMA_RX_I_2_REG RX Copy DMA permission control register 2. 0x00A4 varies
PMS_DMA_RX_I_3_REG RX Copy DMA status register. 0x00A8 RO
PMS_DMA_TX_I_2_REG TX Copy DMA permission control register 2. 0x00B4 varies
PMS_DMA_TX_I_3_REG TX Copy DMA status register. 0x00B8 RO
PMS_APB_PERIPHERAL_INTR_REG PeriBus2 permission control register. 0x00F4 varies
PMS_APB_PERIPHERAL_STATUS_REG PeriBus2 peripheral access status register. 0x00F8 RO
PMS_CPU_PERIPHERAL_INTR_REG PeriBus1 permission control register. 0x00FC varies
PMS_CPU_PERIPHERAL_STATUS_REG PeriBus1 peripheral access status register. 0x0100 RO
Version Control Register
PMS_DATE Version control register. 0x0FFC R/W


*/

/*
HMAC
PeriBUS1 0x3F43D000
PeriBUS2 0x6003D000
*/
/* ULP

26.4 ULP Coprocessor Workflow
ULP coprocessor is designed to operate independently of the CPU, while the CPU is either in sleep or
running.


In a typical power-saving scenario, the chip goes to Deep-sleep mode to lower power consumption. Before
setting the chip to sleep mode, users should complete the following operations.
1. Flash the program to be executed by ULP coprocessor into RTC slow memory.
2. Select the working ULP coprocessor by configuring the register RTC_CNTL_COCPU_SEL.
• 0: select ULP-RISCV
• 1: select ULP-FSM

3. Set sleep cycles for the timer by configuring RTC_CNTL_ULP_CP_TIMER_1_REG.

4. Enable the timer by software or by RTC GPIO;
• By software: set the register RTC_CNTL_ULP_CP_SLP_TIMER_EN.
• By RTC GPIO: set the register RTC_CNTL_ULP_CP_GPIO_WAKEUP_ENA.

5. Set the system into sleep mode.


When the system is in Deep-sleep mode:
1. The timer periodically sets the low-power controller (see Chapter Low Power Management) to Monitor
mode and then wakes up the coprocessor.
2. Coprocessor executes some necessary operations, such as monitoring external environment via
low-power sensors.
3. After the operations are finished, the system goes back to Deep-sleep mode.
4. ULP coprocessor goes back to halt mode and waits for next wakeup.

In monitor mode, ULP coprocessor is woken up and goes to halt as shown in Figure 26-4.
Figure 26-4. Sample of a ULP Operation Sequence
1. Enable the timer and the timer starts counting.
2. The timer expires and wakes up the ULP coprocessor. ULP coprocessor starts running and executes the
program flashed in RTC slow memory.

3. ULP coprocessor goes to halt and the timer starts counting again.
• Put ULP-RISCV into HALT: set the register RTC_CNTL_COCPU_DONE,
• Put ULP-FSM into HALT: execute HALT instruction.

4. Disable the timer by ULP program or by software. The system exits from monitor mode.
• Disabled by software: clear the register RTC_CNTL_ULP_CP_SLP_TIMER_EN.
• Disabled by RTC GPIO: clear the register RTC_CNTL_ULP_CP_GPIO_WAKEUP_ENA, and set the
register RTC_CNTL_ULP_CP_GPIO_WAKEUP_CLR.

Note:
• If the timer is enabled by software (RTC GPIO), it should be disabled by software (RTC GPIO).
• Before setting ULP-RISCV to HALT, users should configure the register RTC_CNTL_COCPU_DONE first,

therefore, it is recommended to end the flashed program with the following pattern:
– Set the register RTC_CNTL_COCPU_DONE to end the operation of ULP-RISCV and put it into halt;
– Set the register RTC_CNTL_COCPU_SHUT_RESET_EN to reset ULP-RISCV.
Enough time is reserved for the ULP-RISCV to complete the operations above before it goes to halt.
The relationship between the signals and registers is shown in Figure 26-5.


 RV32IMC



*/

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


#define ADDR_RTCSLOW 0x60021000
#define ADDR_FIFO_UART0 0x60000000
#define ADDR_FIFO_UART1 0x60010000
#define ADDR_FIFO_UART2 0x6002E000
#define ADDR_FIFO_I2S0 0x6000F004
#define ADDR_FIFO_I2S1 0x6002D004
#define ADDR_FIFO_RMT_CH0 0x60016000
#define ADDR_FIFO_RMT_CH1 0x60016004
#define ADDR_FIFO_RMT_CH2 0x60016008
#define ADDR_FIFO_RMT_CH3 0x6001600C
#define ADDR_FIFO_I2C_EXT0 0x6001301C
#define ADDR_FIFO_I2C_EXT1 0x6002701C
#define ADDR_FIFO_USB_0 0x60080020
#define ADDR_FIFO_USB_1_L 0x60081000
#define ADDR_FIFO_USB_1_H 0x60090FFF

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

