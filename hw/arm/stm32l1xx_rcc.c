/*
 * STM32 Microcontroller RCC (Reset and Clock Control) module
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Source code based on omap_clk.c
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <inttypes.h>
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "stm32l1xx_rcc.h"
#include "qemu/timer.h"
#include <stdio.h>


/* DEFINITIONS*/

/* See README for DEBUG details. */
#define DEBUG_STM32_RCC 1

#ifdef DEBUG_STM32_RCC
#define DPRINTF(fmt, ...)                                       \
do { printf("STM32F2XX_RCC: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define IS_RESET_VALUE(new_value, mask, reset_value) ((new_value & mask) == (mask & reset_value))

#define WARN_UNIMPLEMENTED(new_value, mask, reset_value) \
    if (!IS_RESET_VALUE(new_value, mask, reset_value)) { \
        stm32_unimp("Not implemented: RCC " #mask ". Masked value: 0x%08x\n", (new_value & mask)); \
    }

#define WARN_UNIMPLEMENTED_REG(offset) \
        stm32_unimp("STM32f2xx_rcc: unimplemented register: 0x%x", (int)offset)

#define HSI_FREQ 16000000
#define MSI_FREQ 16000000
#define LSI_FREQ 32000

#define RCC_CR_RESET_VALUE      0x00000083
#define RCC_CR_OFFSET           0x00
#define RCC_CR_RTCPRE1_BIT      30
#define RCC_CR_RTCPRE0_BIT      29
#define RCC_CR_CSSON_BIT        28
#define RCC_CR_PLLRDY_BIT       25
#define RCC_CR_PLLON_BIT        24
#define RCC_CR_HSEBYP_BIT       18
#define RCC_CR_HSERDY_BIT       17
#define RCC_CR_HSEON_BIT        16
#define RCC_CR_MSIRDY_BIT       9
#define RCC_CR_MSION_BIT        8
#define RCC_CR_HSIRDY_BIT       1
#define RCC_CR_HSION_BIT        0



#define RCC_ICSCR_OFFSET        0x04

/*
#define RCC_PLLCFGR_RESET_VALUE  0x24003010
#define RCC_PLLCFGR_OFFSET       0x04
#define RCC_PLLCFGR_PLLQ_MASK    0x0F000000
#define RCC_PLLCFGR_PLLQ_START   24
#define RCC_PLLCFGR_PLLSRC_BIT   22
#define RCC_PLLCFGR_PLLP_MASK    0x00030000
#define RCC_PLLCFGR_PLLP_START   16
#define RCC_PLLCFGR_PLLN_MASK    0x00007FC0
#define RCC_PLLCFGR_PLLN_START   6
#define RCC_PLLCFGR_PLLM_MASK    0x0000003F
#define RCC_PLLCFGR_PLLM_START   0
*/

#define RCC_CFGR_RESET_VALUE     0x00000000
#define RCC_CFGR_OFFSET          0x08
#define RCC_CFGR_MCO2_START      30
#define RCC_CFGR_MCO2_MASK       0xC0000000
#define RCC_CFGR_MCO2PRE_START   27
#define RCC_CFGR_MCO2PRE_MASK    0x38000000
#define RCC_CFGR_MCO1PRE_START   24
#define RCC_CFGR_MCO1PRE_MASK    0x07000000
#define RCC_CFGR_I2CSRC_BIT      23
#define RCC_CFGR_MCO1_START      21
#define RCC_CFGR_MCO1_MASK       0x00600000
#define RCC_CFGR_RTCPRE_START    16
#define RCC_CFGR_RTCPRE_MASK     0x001F0000
#define RCC_CFGR_PPRE2_START     13
#define RCC_CFGR_PPRE2_MASK      0x0000E000
#define RCC_CFGR_PPRE1_START     10
#define RCC_CFGR_PPRE1_MASK      0x00001C00
#define RCC_CFGR_HPRE_START      4
#define RCC_CFGR_HPRE_MASK       0x000000F0
#define RCC_CFGR_SWS_START       2
#define RCC_CFGR_SWS_MASK        0x0000000C
#define RCC_CFGR_SW_START        0
#define RCC_CFGR_SW_MASK         0x00000003

#define RCC_CIR_RESET_VALUE      0x00000000
#define RCC_CIR_OFFSET           0x0C
#define RCC_CIR_CSSC_BIT         23
#define RCC_CIR_PLLI2SRDYC_BIT   21
#define RCC_CIR_PLLRDYC_BIT      20
#define RCC_CIR_HSERDYC_BIT      19
#define RCC_CIR_HSIRDYC_BIT      18
#define RCC_CIR_LSERDYC_BIT      17
#define RCC_CIR_LSIRDYC_BIT      16
#define RCC_CIR_PLLI2SRDYIE_BIT  13
#define RCC_CIR_PLLRDYIE_BIT     12
#define RCC_CIR_HSERDYIE_BIT     11
#define RCC_CIR_HSIRDYIE_BIT     10
#define RCC_CIR_LSERDYIE_BIT     9
#define RCC_CIR_LSIRDYIE_BIT     8
#define RCC_CIR_CSSF_BIT         7
#define RCC_CIR_PLLI2SRDYF_BIT  5
#define RCC_CIR_PLLRDYF_BIT     4
#define RCC_CIR_HSERDYF_BIT     3
#define RCC_CIR_HSIRDYF_BIT     2
#define RCC_CIR_LSERDYF_BIT     1
#define RCC_CIR_LSIRDYF_BIT     0

#define RCC_AHBRSTR_OFFSET  0x10

#define RCC_AHB2RSTR_OFFSET 0x14
#define RCC_APB2RSTR_RESET_VALUE     0x00000000
#define RCC_APB2RSTR_TIM11RST_BIT    18
#define RCC_APB2RSTR_TIM10RST_BIT    17
#define RCC_APB2RSTR_TIM9RST_BIT     16
#define RCC_APB2RSTR_SYSCFGRST       14
#define RCC_APB2RSTR_SPI1_BIT        12
#define RCC_APB2RSTR_SDIORST_BIT     11
#define RCC_APB2RSTR_ADCRST_BIT      8
#define RCC_APB2RSTR_USART6RST       5
#define RCC_APB2RSTR_USART1RST       4
#define RCC_APB2RSTR_TIM8RST_BIT     1
#define RCC_APB2RSTR_TIM1RST_BIT     0


#define RCC_AHB1RSTR_OFFSET 0x18

#define RCC_AHBENR_OFFSET   0x1C

#define RCC_APB2ENR_OFFSET  0x20
/*
#define RCC_AHB2ENR_OFFSET 0x34
#define RCC_AHB2ENR_DCMIEN_BIT       0
#define RCC_AHB2ENR_CRYPEN_BIT       4
#define RCC_AHB2ENR_HASHEN_BIT       5
#define RCC_AHB2ENR_RNGEN_BIT        6
#define RCC_AHB2ENR_OTGFSEN_BIT      7
*/

#define RCC_APB1ENR_RESET_VALUE      0x00000000
#define RCC_APB1ENR_OFFSET           0x24
#define RCC_APB1ENR_COMPEN           31
//#define RCC_APB1ENR_USART7EN_BIT 30
#define RCC_APB1ENR_DACEN_BIT        29
// RCC, power enable bit...
#define RCC_APB1ENR_PWREN_BIT        28
#define RCC_APB1ENR_BKPEN_BIT        27
//#define RCC_APB1ENR_CAN2EN_BIT   26
//#define RCC_APB1ENR_CAN1EN_BIT   25
//#define RCC_APB1ENR_CANEN_BIT    25
#define RCC_APB1ENR_USBEN_BIT    23
#define RCC_APB1ENR_I2C2EN_BIT   22
#define RCC_APB1ENR_I2C1EN_BIT   21
#define RCC_APB1ENR_USART5EN_BIT 20
#define RCC_APB1ENR_USART4EN_BIT 19
#define RCC_APB1ENR_USART3EN_BIT 18
#define RCC_APB1ENR_USART2EN_BIT 17
#define RCC_APB1ENR_SPI3EN_BIT   15
#define RCC_APB1ENR_SPI2EN_BIT   14
#define RCC_APB1ENR_WWDGEN_BIT   11
#define RCC_APB1ENR_TIM7EN_BIT   5
#define RCC_APB1ENR_TIM6EN_BIT   4
#define RCC_APB1ENR_TIM5EN_BIT   3
#define RCC_APB1ENR_TIM4EN_BIT   2
#define RCC_APB1ENR_TIM3EN_BIT   1
#define RCC_APB1ENR_TIM2EN_BIT   0

// WAS RCC_AHB1ENR_
#define RCC_AHBLPENR_RESET_VALUE      0x00000000
#define RCC_AHBLPENR_OFFSET  0x28
#define RCC_AHBLPENR_OTGHSULPIEN_BIT  30
#define RCC_AHBLPENR_OTGHSEN_BIT      29
#define RCC_AHBLPENR_ETHMACPTPEN_BIT  28
#define RCC_AHBLPENR_ETHMACRXEN_BIT   27
#define RCC_AHBLPENR_ETHMACTXEN_BIT   26
#define RCC_AHBLPENR_ETHMACEN_BIT     25
#define RCC_AHBLPENR_DMA2EN_BIT       22
#define RCC_AHBLPENR_DMA1EN_BIT       21
#define RCC_AHBLPENR_BKPSRAMEN_BIT    18
#define RCC_AHBLPENR_CRCEN_BIT        12
#define RCC_AHBLPENR_GPIOKEN_BIT      10
#define RCC_AHBLPENR_GPIOJEN_BIT      9
#define RCC_AHBLPENR_GPIOIEN_BIT      8
#define RCC_AHBLPENR_GPIOHEN_BIT      7
#define RCC_AHBLPENR_GPIOGEN_BIT      6
#define RCC_AHBLPENR_GPIOFEN_BIT      5
#define RCC_AHBLPENR_GPIOEEN_BIT      4
#define RCC_AHBLPENR_GPIODEN_BIT      3
#define RCC_AHBLPENR_GPIOCEN_BIT      2
#define RCC_AHBLPENR_GPIOBEN_BIT      1
#define RCC_AHBLPENR_GPIOAEN_BIT      0


#define RCC_APB2LPENR_OFFSET  0x2C
#define RCC_APB2LPENR_USART1LPEN_BIT 14
#define RCC_APB2LPENR_SPI1LPEN_BIT   12
#define RCC_APB2LPENR_SDIOLPEN_BIT   11
#define RCC_APB2ENR_ADC1LPEN_BIT     9




#define RCC_CSR_RESET_VALUE        0x0E000000
#define RCC_CSR_OFFSET             0x34
#define RCC_CSR_LPWRRSTF_BIT       31
#define RCC_CSR_WWDGRSTF_BIT       30
#define RCC_CSR_IWDGRSTF_BIT       29
#define RCC_CSR_SFTRSTF_BIT        28
#define RCC_CSR_PORRSTF_BIT        27
#define RCC_CSR_PINRSTF_BIT        26
#define RCC_CSR_BORRSTF_BIT        25
#define RCC_CSR_RMVF_BIT           24
#define RCC_CSR_LSERDY_BIT         9
#define RCC_CSR_LSEON_BIT          8
#define RCC_CSR_LSIRDY_BIT         1
#define RCC_CSR_LSION_BIT          0



/*
-----------------------------




#define RCC_AHB3ENR_OFFSET 0x38
#define RCC_AHB3ENR_FSMCEN_BIT       0


#define RCC_APB2ENR_RESET_VALUE  0x00000000
#define RCC_APB2ENR_OFFSET       0x44
#define RCC_APB2ENR_MASK         0x00075F33
#define RCC_APB2ENR_TIM11EN      18
#define RCC_APB2ENR_TIM10EN      17
#define RCC_APB2ENR_TIM9EN       16
#define RCC_APB2ENR_SYSCFGEN_BIT 14
#define RCC_APB2ENR_SPI1EN_BIT   12
#define RCC_APB2ENR_SDIOEN_BIT   11
#define RCC_APB2ENR_ADC3EN_BIT   10
#define RCC_APB2ENR_ADC2EN_BIT   9
#define RCC_APB2ENR_ADC1EN_BIT   8
#define RCC_APB2ENR_USART6EN_BIT 5
#define RCC_APB2ENR_USART1EN_BIT 4
#define RCC_APB2ENR_TIM8EN_BIT   1
#define RCC_APB2ENR_TIM1EN_BIT   0

#define RCC_AHB1LPENR_RESET_VALUE  0x7E6791FF
#define RCC_AHB1LPENR_OFFSET       0x50

#define RCC_AHB2LPENR_RESET_VALUE  0x000000F1
#define RCC_AHB2LPENR_OFFSET       0x54

#define RCC_AHB3LPENR_RESET_VALUE  0x00000001
#define RCC_AHB3LPENR_OFFSET       0x58

#define RCC_APB1LPENR_RESET_VALUE  0x36FEC9FF
#define RCC_APB1LPENR_OFFSET       0x60

#define RCC_APB2LPENR_RESET_VALUE  0x00075F33
#define RCC_APB2LPENR_OFFSET       0x64

#define RCC_BDCR_RESET_VALUE       0x00000000
#define RCC_BDCR_OFFSET            0x70
#define RCC_BDCR_BDRST_BIT         16
#define RCC_BDCR_RTCEN_BIT         15
#define RCC_BDCR_RTCSEL_START      8
#define RCC_BDCR_RTCSEL_MASK       0x00000300
#define RCC_BDCR_LSEBYP_BIT        2
#define RCC_BDCR_LSERDY_BIT        1
#define RCC_BDCR_LSEON_BIT         0

#define RCC_SSCGR_RESET_VALUE      0x00000000
#define RCC_SSCGR_OFFSET           0x80

#define RCC_PLLI2SCFGR_RESET_VALUE 0x20003000
#define RCC_PLLI2SCFGR_OFFSET      0x84
#define RCC_PLLI2SCFGR_PLLR_MASK    0x70000000
#define RCC_PLLI2SCFGR_PLLR_START   28
#define RCC_PLLI2SCFGR_PLLQ_MASK    0x0F000000
#define RCC_PLLI2SCFGR_PLLQ_START   24
#define RCC_PLLI2SCFGR_PLLN_MASK    0x00007FC0
#define RCC_PLLI2SCFGR_PLLN_START   6
*/


#define PLLSRC_HSI_SELECTED 0
#define PLLSRC_HSE_SELECTED 1

#define SW_HSI_SELECTED 0
#define SW_HSE_SELECTED 1
#define SW_PLL_SELECTED 2

/* HELPER FUNCTIONS */
struct Clk {                                                                    
    const char *name;                                                           
                                                                                
    bool enabled;                                                               
                                                                                
    uint32_t input_freq, output_freq, max_output_freq;                          
                                                                                
    uint16_t multiplier, divisor;                                               
                                                                                
    unsigned user_count;                                                        
    qemu_irq user[CLKTREE_MAX_IRQ]; /* Who to notify on change */               
                                                                                
    unsigned output_count;                                                      
    struct Clk *output[CLKTREE_MAX_OUTPUT];                                     
                                                                                
    unsigned input_count;                                                       
    int selected_input;                                                         
    struct Clk *input[CLKTREE_MAX_INPUT];                                       
};                                                                              
                                                                                

/* Enable the peripheral clock if the specified bit is set in the value. */
static void stm32_rcc_periph_enable(
                                    Stm32lixxRcc *s,
                                    uint32_t new_value,
                                    bool init,
                                    int periph,
                                    uint32_t bit_mask)
{
    //printf("rcc set 0x%x %s %d %x: %sable\n", new_value, s->PERIPHCLK[periph]->name, periph, bit_mask, IS_BIT_SET(new_value, bit_mask)?"en":"dis");
    clktree_set_enabled(s->PERIPHCLK[periph], IS_BIT_SET(new_value, bit_mask));
}





/* REGISTER IMPLEMENTATION */

/* Read the configuration register.
   NOTE: Not implemented: CSS, PLLI2S, clock calibration and trimming. */
static uint32_t stm32_rcc_RCC_CR_read(Stm32lixxRcc *s)
{
    /* Get the status of the clocks. */
    const bool PLLON = clktree_is_enabled(s->PLLCLK);
    const bool HSEON = clktree_is_enabled(s->HSECLK);
    const bool MSION = clktree_is_enabled(s->MSICLK);
    const bool PLLI2SON = clktree_is_enabled(s->PLLI2SCLK);

    /* build the register value based on the clock states.  If a clock is on,
     * then its ready bit is always set.
     */
    return (
        GET_BIT_MASK(RCC_CR_PLLRDY_BIT, PLLON) |
        GET_BIT_MASK(RCC_CR_PLLON_BIT, PLLON) |

        GET_BIT_MASK(RCC_CR_HSERDY_BIT, HSEON) |
        GET_BIT_MASK(RCC_CR_HSEON_BIT, HSEON) |

        GET_BIT_MASK(RCC_CR_MSIRDY_BIT, MSION) |
        GET_BIT_MASK(RCC_CR_MSION_BIT, MSION) |

        GET_BIT_MASK(RCC_CR_PLLRDY_BIT, PLLI2SON) |
        GET_BIT_MASK(RCC_CR_PLLON_BIT, PLLI2SON)
    );
}

/* Write the Configuration Register.
 * This updates the states of the corresponding clocks.  The bit values are not
 * saved - when the register is read, its value will be built using the clock
 * states.
 */
static void stm32_rcc_RCC_CR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    bool new_PLLON, new_HSEON, new_HSION,new_MSION, new_PLLI2SON;

    new_PLLON = IS_BIT_SET(new_value, RCC_CR_PLLON_BIT);
    if((clktree_is_enabled(s->PLLCLK) && !new_PLLON) &&
       s->RCC_CFGR_SW == SW_PLL_SELECTED) {
        stm32_hw_warn("PLL cannot be disabled while it is selected as the system clock.");
    }
    clktree_set_enabled(s->PLLCLK, new_PLLON);

    new_HSEON = IS_BIT_SET(new_value, RCC_CR_HSEON_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSE_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        stm32_hw_warn("HSE oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSECLK, new_HSEON);

    new_HSION = IS_BIT_SET(new_value, RCC_CR_HSION_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSI_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        stm32_hw_warn("HSI oscillator cannot be disabled while it is driving the system clock.");
    }
    clktree_set_enabled(s->HSICLK, new_HSION);


    new_MSION = IS_BIT_SET(new_value, RCC_CR_MSION_BIT);
    if((clktree_is_enabled(s->HSECLK) && !new_HSEON) &&
       (s->RCC_CFGR_SW == SW_HSI_SELECTED || s->RCC_CFGR_SW == SW_PLL_SELECTED)
       ) {
        stm32_hw_warn("HSI oscillator cannot be disabled while it is driving the system clock.");
    }

    clktree_set_enabled(s->MSICLK, new_MSION);

    new_PLLI2SON = IS_BIT_SET(new_value, RCC_CR_PLLON_BIT);
    clktree_set_enabled(s->PLLI2SCLK, new_PLLI2SON);

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CR_CSSON_BIT, RCC_CR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, RCC_CR_HSICAL_MASK, RCC_CR_RESET_VALUE);
    //WARN_UNIMPLEMENTED(new_value, RCC_CR_HSITRIM_MASK, RCC_CR_RESET_VALUE);
}





static uint32_t stm32_rcc_RCC_CFGR_read(Stm32lixxRcc *s)
{
    return
    (s->RCC_CFGR_PPRE2 << RCC_CFGR_PPRE2_START) |
    (s->RCC_CFGR_PPRE1 << RCC_CFGR_PPRE1_START) |
    (s->RCC_CFGR_HPRE << RCC_CFGR_HPRE_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SW_START) |
    (s->RCC_CFGR_SW << RCC_CFGR_SWS_START);
}

static void stm32_rcc_RCC_CFGR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    /* PPRE2 */
    s->RCC_CFGR_PPRE2 = (new_value & RCC_CFGR_PPRE2_MASK) >> RCC_CFGR_PPRE2_START;
    if(s->RCC_CFGR_PPRE2 < 0x4) {
        clktree_set_scale(s->PCLK2, 1, 1);
    } else {
        clktree_set_scale(s->PCLK2, 1, 2 * (s->RCC_CFGR_PPRE2 - 3));
    }

    /* PPRE1 */
    s->RCC_CFGR_PPRE1 = (new_value & RCC_CFGR_PPRE1_MASK) >> RCC_CFGR_PPRE1_START;
    if(s->RCC_CFGR_PPRE1 < 4) {
        clktree_set_scale(s->PCLK1, 1, 1);
    } else {
        clktree_set_scale(s->PCLK1, 1, 2 * (s->RCC_CFGR_PPRE1 - 3));
    }

    /* HPRE */
    s->RCC_CFGR_HPRE = (new_value & RCC_CFGR_HPRE_MASK) >> RCC_CFGR_HPRE_START;
    if(s->RCC_CFGR_HPRE < 8) {
        clktree_set_scale(s->HCLK, 1, 1);
    } else {
        clktree_set_scale(s->HCLK, 1, 2 * (s->RCC_CFGR_HPRE - 7));
    }

    /* SW */
    s->RCC_CFGR_SW = (new_value & RCC_CFGR_SW_MASK) >> RCC_CFGR_SW_START;
    switch(s->RCC_CFGR_SW) {
        case 0x0:
        case 0x1:
        case 0x2:
            clktree_set_selected_input(s->SYSCLK, s->RCC_CFGR_SW);
            break;
        default:
            hw_error("Invalid input selected for SYSCLK");
            break;
    }

    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO2PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1PRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CFGR_I2CSRC_BIT, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_MCO1_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_RTCPRE_MASK, RCC_CFGR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_CFGR_SWS_MASK, RCC_CFGR_RESET_VALUE);
}

static uint32_t stm32_rcc_RCC_CIR_read(Stm32lixxRcc *s)
{
    return s->RCC_CIR;
}

static void stm32_rcc_RCC_CIR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_CSSC_BIT, RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYC_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYIE_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_CSSF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLI2SRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_PLLRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSERDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_HSIRDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSERDYF_BIT , RCC_CIR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_CIR_LSIRDYF_BIT , RCC_CIR_RESET_VALUE);
}

static uint32_t stm32_rcc_RCC_AHBLPENR_read(Stm32lixxRcc *s)
{
    return s->RCC_AHBLPENR;
}

static uint32_t stm32_rcc_RCC_AHB2ENR_read(Stm32lixxRcc *s)
{
    return s->RCC_AHB2ENR;
}

static uint32_t stm32_rcc_RCC_AHB3ENR_read(Stm32lixxRcc *s)
{
    return s->RCC_AHB3ENR;
}

static void stm32_rcc_RCC_AHBLPENR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->PERIPHCLK[STM32_DMA2], IS_BIT_SET(new_value, RCC_AHBLPENR_DMA2EN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_DMA1], IS_BIT_SET(new_value, RCC_AHBLPENR_DMA1EN_BIT));

    clktree_set_enabled(s->PERIPHCLK[STM32_CRC], IS_BIT_SET(new_value, RCC_AHBLPENR_CRCEN_BIT));

    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOK], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOKEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOJ], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOJEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOI], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOIEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOH], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOHEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOG], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOGEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOF], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOFEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOE], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOEEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOD], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIODEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOC], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOCEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOB], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOBEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_GPIOA], IS_BIT_SET(new_value, RCC_AHBLPENR_GPIOAEN_BIT));

    s->RCC_AHBLPENR = new_value;

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHBLPENR_OTGHSULPIEN_BIT, RCC_AHBLPENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHBLPENR_OTGHSEN_BIT, RCC_AHBLPENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHBLPENR_ETHMACPTPEN_BIT, RCC_AHBLPENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHBLPENR_ETHMACRXEN_BIT, RCC_AHBLPENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHBLPENR_ETHMACTXEN_BIT, RCC_AHBLPENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHBLPENR_ETHMACEN_BIT, RCC_AHBLPENR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_AHBLPENR_BKPSRAMEN_BIT, RCC_AHBLPENR_RESET_VALUE);
}
/*
static void stm32_rcc_RCC_AHB2ENR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->PERIPHCLK[STM32_DCMI_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_DCMIEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_CRYP_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_CRYPEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_HASH_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_HASHEN_BIT));
    clktree_set_enabled(s->PERIPHCLK[STM32_RNG_PERIPH],
                        IS_BIT_SET(new_value, RCC_AHB2ENR_RNGEN_BIT));
    s->RCC_AHB2ENR = new_value;
}

static void stm32_rcc_RCC_AHB3ENR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->PERIPHCLK[STM32_FSMC],
                        IS_BIT_SET(new_value, RCC_AHB3ENR_FSMCEN_BIT));
    s->RCC_AHB3ENR = new_value;
}
*/

/* Write the APB2 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB2LPENR_write(Stm32lixxRcc *s, uint32_t new_value,
                                        bool init)
{
    /* TODO: enable/disable missing peripherals */
    //stm32_rcc_periph_enable(s, new_value, init, STM32_SYSCFG, RCC_APB2ENR_SYSCFGEN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART1, RCC_APB2LPENR_USART1LPEN_BIT);

    s->RCC_APB2LPENR = new_value; // & RCC_APB2ENR_MASK;
}

/* Write the APB1 peripheral clock enable register
 * Enables/Disables the peripheral clocks based on each bit. */
static void stm32_rcc_RCC_APB1ENR_write(Stm32lixxRcc *s, uint32_t new_value,
                                        bool init)
{
    /*
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART8,
                            RCC_APB1ENR_USART8EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART7,
                            RCC_APB1ENR_USART7EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART5,
                            RCC_APB1ENR_USART5EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART4,
                            RCC_APB1ENR_USART4EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART3,
                            RCC_APB1ENR_USART3EN_BIT);
    stm32_rcc_periph_enable(s, new_value, init, STM32_UART2,
                            RCC_APB1ENR_USART2EN_BIT);
*/
    /* 0b00110110111111101100100111111111 */
    s->RCC_APB1ENR = new_value & 0x36fec9ff;
}

/*
static uint32_t stm32_rcc_RCC_BDCR_read(Stm32lixxRcc *s)
{
    bool lseon = clktree_is_enabled(s->LSECLK);

    return GET_BIT_MASK(RCC_BDCR_LSERDY_BIT, lseon) |
    GET_BIT_MASK(RCC_BDCR_LSEON_BIT, lseon) | 0x100; // XXX force LSE
}

static void stm32_rcc_RCC_BDCR_writeb0(Stm32lixxRcc *s, uint8_t new_value, bool init)
{
    clktree_set_enabled(s->LSECLK, IS_BIT_SET(new_value, RCC_BDCR_LSEON_BIT));
    
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_LSEBYP_BIT, RCC_BDCR_RESET_VALUE);
}

static void stm32_rcc_RCC_BDCR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    stm32_rcc_RCC_BDCR_writeb0(s, new_value & 0xff, init);

    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_BDRST_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, 1 << RCC_BDCR_RTCEN_BIT, RCC_BDCR_RESET_VALUE);
    WARN_UNIMPLEMENTED(new_value, RCC_BDCR_RTCSEL_MASK, RCC_BDCR_RESET_VALUE);
}
*/

/* Works the same way as stm32_rcc_RCC_CR_read */
static uint32_t stm32_rcc_RCC_CSR_read(Stm32lixxRcc *s)
{
    //bool hseon = clktree_is_enabled(s->HSECLK);

    //return GET_BIT_MASK(RCC_CR_HSERDY_BIT, hseon) |
    //GET_BIT_MASK(RCC_CR_HSEON_BIT, hseon);
    // Fake ready and on internal external low speed
    bool hseon=true;
    return GET_BIT_MASK(RCC_CSR_LSIRDY_BIT, hseon) |
      GET_BIT_MASK(RCC_CSR_LSION_BIT, hseon) |
      GET_BIT_MASK(RCC_CSR_LSERDY_BIT, hseon) |
      GET_BIT_MASK(RCC_CSR_LSEON_BIT, hseon);

}

/* Works the same way as stm32_rcc_RCC_CR_write */
static void stm32_rcc_RCC_CSR_write(Stm32lixxRcc *s, uint32_t new_value, bool init)
{
    clktree_set_enabled(s->LSECLK, IS_BIT_SET(new_value, RCC_CSR_LSEON_BIT));
}



static uint64_t stm32_rcc_readw(void *opaque, hwaddr offset)
{
    Stm32lixxRcc *s = (Stm32lixxRcc *)opaque;

    switch (offset) {
        case RCC_CR_OFFSET:
            return stm32_rcc_RCC_CR_read(s);
        case RCC_ICSCR_OFFSET:
            return (s->RCC_ICSCR);
        //case RCC_PLLCFGR_OFFSET:
        //    return stm32_rcc_RCC_PLLCFGR_read(s);
        case RCC_CFGR_OFFSET:
            return stm32_rcc_RCC_CFGR_read(s);
        case RCC_CIR_OFFSET:
            return stm32_rcc_RCC_CIR_read(s);
        case RCC_AHBRSTR_OFFSET:
        case RCC_AHB2RSTR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            return 0;
            return 0;
        case RCC_AHBLPENR_OFFSET:
            return stm32_rcc_RCC_AHBLPENR_read(s);
        case RCC_APB1ENR_OFFSET:
            return s->RCC_APB1ENR;
        case RCC_APB2ENR_OFFSET:
            return s->RCC_APB2ENR;
        case RCC_APB2LPENR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            return 0;
        case RCC_CSR_OFFSET:
            return stm32_rcc_RCC_CSR_read(s);
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
    return 0;
}


static void stm32_rcc_writew(void *opaque, hwaddr offset,
                             uint64_t value)
{
    Stm32lixxRcc *s = (Stm32lixxRcc *)opaque;

    switch(offset) {
        case RCC_CR_OFFSET:
            stm32_rcc_RCC_CR_write(s, value, false);
            break;
        case RCC_CFGR_OFFSET:
            stm32_rcc_RCC_CFGR_write(s, value, false);
            break;
        case RCC_CIR_OFFSET:
            stm32_rcc_RCC_CIR_write(s, value, false);
            break;
        case RCC_AHBLPENR_OFFSET:
            stm32_rcc_RCC_AHBLPENR_write(s, value, false);
            break;
        case RCC_APB2LPENR_OFFSET:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
        case RCC_APB1ENR_OFFSET:
            stm32_rcc_RCC_APB1ENR_write(s, value, false);
            break;
        case RCC_CSR_OFFSET:
            stm32_rcc_RCC_CSR_write(s, value, false);
            break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
}

static uint64_t stm32_rcc_read(void *opaque, hwaddr offset,
                               unsigned size)
{
    switch(size) {
        case 4:
            return stm32_rcc_readw(opaque, offset);
        default:
            stm32_unimp("Unimplemented: RCC read from register at offset %lld", (long long) offset);
            return 0;
    }
}

static void stm32_rcc_write(void *opaque, hwaddr offset,
                            uint64_t value, unsigned size)
{
    switch(size) {
        case 4:
            stm32_rcc_writew(opaque, offset, value);
            break;
        default:
            WARN_UNIMPLEMENTED_REG(offset);
            break;
    }
}

static const MemoryRegionOps stm32_rcc_ops = {
    .read = stm32_rcc_read,
    .write = stm32_rcc_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};


static void stm32_rcc_reset(DeviceState *dev)
{
    Stm32lixxRcc *s = FROM_SYSBUS(Stm32lixxRcc, SYS_BUS_DEVICE(dev));

    stm32_rcc_RCC_CR_write(s, RCC_CR_RESET_VALUE, true);
    //stm32_rcc_RCC_PLLCFGR_write(s, RCC_PLLCFGR_RESET_VALUE, true);
    stm32_rcc_RCC_CFGR_write(s, RCC_CFGR_RESET_VALUE, true);
    //stm32_rcc_RCC_APB2LPENR_write(s, RCC_APB2ENR_RESET_VALUE, true);
    stm32_rcc_RCC_APB1ENR_write(s, RCC_APB1ENR_RESET_VALUE, true);
    stm32_rcc_RCC_CSR_write(s, RCC_CSR_RESET_VALUE, true);
}

/* IRQ handler to handle updates to the HCLK frequency.
 * This updates the SysTick scales. */
static void stm32_rcc_hclk_upd_irq_handler(void *opaque, int n, int level)
{
    Stm32lixxRcc *s = (Stm32lixxRcc *)opaque;

    uint32_t hclk_freq = 0;
    //uint32_t ext_ref_freq = 0;
    
    hclk_freq = clktree_get_output_freq(s->HCLK);

    /* Only update the scales if the frequency is not zero. */
    if (hclk_freq > 0) {
        //ext_ref_freq = hclk_freq / 8;

        /* Update the scales - these are the ratio of QEMU clock ticks
         * (which is an unchanging number independent of the CPU frequency) to
         * system/external clock ticks.
         */
        system_clock_scale = NANOSECONDS_PER_SECOND /*get_ticks_per_sec()*/ / hclk_freq;
    }

#ifdef DEBUG_STM32_RCC
    DPRINTF("Cortex SYSTICK frequency set to %lu Hz (scale set to %d).\n",
            (unsigned long)hclk_freq, system_clock_scale);
#endif
}



/* DEVICE INITIALIZATION */

/* Set up the clock tree */
static void stm32_rcc_init_clk(Stm32lixxRcc *s)
{
    int i;
    qemu_irq *hclk_upd_irq =
    qemu_allocate_irqs(stm32_rcc_hclk_upd_irq_handler, s, 1);

    /* Make sure all the peripheral clocks are null initially.
     * This will be used for error checking to make sure
     * an invalid clock is not referenced (not all of the
     * indexes will be used).
     */
    for(i = 0; i < STM32_PERIPH_COUNT; i++) {
        s->PERIPHCLK[i] = NULL;
    }

    /* Initialize clocks */
    /* Source clocks are initially disabled, which represents
     * a disabled oscillator.  Enabling the clock represents
     * turning the clock on.
     */
    s->HSICLK = clktree_create_src_clk("HSI", MSI_FREQ, false);
    s->MSICLK = clktree_create_src_clk("MSI", HSI_FREQ, false);

    s->LSICLK = clktree_create_src_clk("LSI", LSI_FREQ, false);
    s->HSECLK = clktree_create_src_clk("HSE", s->osc_freq, false);
    s->LSECLK = clktree_create_src_clk("LSE", s->osc32_freq, false);

    s->IWDGCLK = clktree_create_clk("IWDGCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0,
                                    s->LSICLK, NULL);
    s->RTCCLK = clktree_create_clk("RTCCLK", 1, 1, false, CLKTREE_NO_MAX_FREQ,
                                   CLKTREE_NO_INPUT, s->LSECLK, s->LSICLK, s->HSECLK, NULL);

    s->PLLM = clktree_create_clk("PLLM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->HSICLK,
                                 s->HSECLK, NULL);
    s->PLLCLK = clktree_create_clk("PLLCLK", 1, 2, false, 120000000, 0, s->PLLM, NULL);
    s->PLL48CLK = clktree_create_clk("PLL48CLK", 1, 1, false, 48000000, 0, s->PLLM, NULL);

    s->PLLI2SM = clktree_create_clk("PLLI2SM", 1, 16, true, CLKTREE_NO_MAX_FREQ, 0, s->PLLM, NULL);
    s->PLLI2SCLK = clktree_create_clk("PLLI2SCLK", 1, 2, false, 120000000, 0, s->PLLI2SM, NULL);

    s->SYSCLK = clktree_create_clk("SYSCLK", 1, 1, true, 168000000, CLKTREE_NO_INPUT,
                                   s->HSICLK, s->HSECLK, s->PLLCLK, NULL);

    // HCLK: to AHB bus, core memory and DMA
    s->HCLK = clktree_create_clk("HCLK", 0, 1, true, 168000000, 0, s->SYSCLK, NULL);
    clktree_adduser(s->HCLK, hclk_upd_irq[0]);

    // Clock source for APB1 peripherals:
    s->PCLK1 = clktree_create_clk("PCLK1", 0, 1, true, 30000000, 0, s->HCLK, NULL);

    // Clock source for APB2 peripherals:
    s->PCLK2 = clktree_create_clk("PCLK2", 0, 1, true, 60000000, 0, s->HCLK, NULL);

    /* Peripheral clocks */
    s->PERIPHCLK[STM32_GPIOA] =
        clktree_create_clk("GPIOA", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOB] =
        clktree_create_clk("GPIOB", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOC] =
        clktree_create_clk("GPIOC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOD] =
        clktree_create_clk("GPIOD", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOE] =
        clktree_create_clk("GPIOE", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOF] =
        clktree_create_clk("GPIOF", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOG] =
        clktree_create_clk("GPIOG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOH] =
        clktree_create_clk("GPIOH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOI] =
        clktree_create_clk("GPIOI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOJ] =
        clktree_create_clk("GPIOJ", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_GPIOK] =
        clktree_create_clk("GPIOK", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);

    s->PERIPHCLK[STM32_CRC] =
        clktree_create_clk("CRC", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);

    s->PERIPHCLK[STM32_DMA1] =
        clktree_create_clk("DMA1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_DMA2] =
        clktree_create_clk("DMA2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    
    s->PERIPHCLK[STM32_SYSCFG] =
        clktree_create_clk("SYSCFG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);

    s->PERIPHCLK[STM32_UART1] =
        clktree_create_clk("UART1", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32_UART2] =
        clktree_create_clk("UART2", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART3] =
        clktree_create_clk("UART3", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART4] =
        clktree_create_clk("UART4", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART5] =
        clktree_create_clk("UART5", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART6] =
        clktree_create_clk("UART6", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK2, NULL);
    s->PERIPHCLK[STM32_UART7] =
        clktree_create_clk("UART7", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);
    s->PERIPHCLK[STM32_UART8] =
        clktree_create_clk("UART8", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->PCLK1, NULL);


    s->PERIPHCLK[STM32_DCMI_PERIPH] =
        clktree_create_clk("DCMI", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_CRYP_PERIPH] =
        clktree_create_clk("CRYP", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_HASH_PERIPH] =
        clktree_create_clk("HASH", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
    s->PERIPHCLK[STM32_RNG_PERIPH] =
        clktree_create_clk("RNG", 1, 1, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);

    s->PERIPHCLK[STM32_FSMC] =
        clktree_create_clk("FSMC", 1, 2, false, CLKTREE_NO_MAX_FREQ, 0, s->HCLK, NULL);
}



static int stm32_rcc_init(SysBusDevice *dev)
{
    Stm32lixxRcc *s = FROM_SYSBUS(Stm32lixxRcc, dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &stm32_rcc_ops, s,
                          "rcc", 0x40023BFF - 0x40023800 + 1);

    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);

    stm32_rcc_init_clk(s);

    return 0;
}


static Property stm32_rcc_properties[] = {
    DEFINE_PROP_UINT32("osc_freq", Stm32lixxRcc, osc_freq, 0),
    DEFINE_PROP_UINT32("osc32_freq", Stm32lixxRcc, osc32_freq, 0),
    DEFINE_PROP_END_OF_LIST()
};


static void stm32_rcc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = stm32_rcc_init;
    dc->reset = stm32_rcc_reset;
    dc->props = stm32_rcc_properties;
}

static TypeInfo stm32_rcc_info = {
    .name  = "stm32l1xx_rcc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(Stm32lixxRcc),
    .class_init = stm32_rcc_class_init
};

static void stm32_rcc_register_types(void)
{
    type_register_static(&stm32_rcc_info);
}

type_init(stm32_rcc_register_types)
