/*
 * ESP32 "DPORT" device
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/misc/esp32s2_reg.h"
#include "hw/misc/esp32s2_dport.h"
#include "target/xtensa/cpu.h"

#define TYPE_ESP32_DPORT "misc.esp32.dport"


#define ESP32_DPORT_SIZE        (DR_REG_DPORT_APB_BASE - DR_REG_DPORT_BASE)

#define MMU_RANGE_SIZE          (ESP32_CACHE_PAGES_PER_REGION * sizeof(uint32_t))
#define MMU_RANGE_LAST          (MMU_RANGE_SIZE - sizeof(uint32_t))

#define PRO_DROM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE)
#define PRO_DROM0_MMU_LAST      (PRO_DROM0_MMU_FIRST + MMU_RANGE_LAST)
#define PRO_IRAM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE + MMU_RANGE_SIZE)
#define PRO_IRAM0_MMU_LAST      (PRO_IRAM0_MMU_FIRST + MMU_RANGE_LAST)
#define APP_DROM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE)
#define APP_DROM0_MMU_LAST      (APP_DROM0_MMU_FIRST + MMU_RANGE_LAST)
#define APP_IRAM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE + MMU_RANGE_SIZE)
#define APP_IRAM0_MMU_LAST      (APP_IRAM0_MMU_FIRST + MMU_RANGE_LAST)
#define MMU_ENTRY_MASK          0x1ff

//static void esp32s2_cache_state_update(Esp32CacheState* cs);
//static void esp32s2_cache_data_sync(Esp32CacheRegionState* crs);

/*
See page 434
ESP32-S2 TRM

MMU Entries Bit Positions Description
SRAM [16]
The external memory attribute indicates whether cache/EDMA accesses the external flash or SRAM. If bit 15 is set, cache/EDMA
accesses flash. If bit 16 is set, cache/EDMA accesses SRAM. The
two bits cannot be set at the same time.
Flash [15]
Invalid [14] Cleared if MMU entry is valid.

Page number [13:0] For MMU purposes, external memory is divided in memory blocks
called ‘pages’. The page size is fixed at 64 KB. The page number
of the physical address space indicates which page is accessed by
cache/EDMA.
When cache or EDMA accesses an invalid MMU page or the external memory attribute is not specified in the
MMU, an MMU error interrupt will be triggered.


*/

/*
 External memory permission control

The hardware divides the physical address of the external memory (flash + SRAM) into eight (4 + 4) areas. Each
area can be individually configured for W/R/X access. Software needs to configure the size of each area and its
access type in advance.
Espressif Systems 434
Submit Documentation Feedback
ESP32-S2 TRM (Preliminary V0.3)
23. Permission Control
The eight areas correspond to eight sets of registers, and the flash and SRAM each has four sets. Each set of
registers contains three parts as follows:
1. Attribute list: APB_CTRL_X_ACS_n_ATTR_REG has 3 bits in total, which are W/R/X bits from high to low;
2. Area start address: APB_CTRL_X_ACS_n_ADDR_REG, which represents the physical address;
3. Area length: APB_CTRL_X_ACS_n_SIZE_REG in multiples of 64 KB;

When loading boot code,
Loading section .dram0.data, size 0x4 lma 0x3ffe8100
Loading section .dram0.rodata, size 0x17d4 lma 0x3ffe8104
Loading section .iram.text, size 0x14b0 lma 0x40050000
Loading section .iram_loader.text, size 0x210b lma 0x40054000
Start address 0x400502d8, load size 19859

Panic at 40013ab7
Fatal exception (15): LoadStorePIFAddrError
epc1=0x4000ff4b, epc2=0x00000000, epc3=0x00000000, excvaddr=0x60080010, depc=0x00000000

#0  0x40013ab7 in dfu_updater_flash_read ()
#1  0x40013ef2 in usb_dc_attach ()
#2  0x40014fe4 in usb_enable ()
#3  0x40013184 in cdc_acm_init ()
#4  0x4001282b in Uart_Init_USB ()
#5  0x4000f5e1 in boot_prepare ()
#6  0x4000f91a in main ()
0x40013ab7 <dfu_updater_flash_read+207> l32i.n a4, a2, 16   
(gdb) p/x $a2
$1 = 0x60080000


*/


static inline uint32_t get_mmu_entry(Esp32CacheRegionState* crs, hwaddr base, hwaddr addr)
{
    return crs->mmu_table[(addr - base)/sizeof(uint32_t)] & MMU_ENTRY_MASK;
}

static inline void set_mmu_entry(Esp32CacheRegionState* crs, hwaddr base, hwaddr addr, uint64_t val)
{
    uint32_t old_val = crs->mmu_table[(addr - base)/sizeof(uint32_t)];
    if (val != old_val) {
        crs->mmu_table[(addr - base)/sizeof(uint32_t)] = (val & MMU_ENTRY_MASK) | ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
}

static uint64_t esp32s2_dport_read(void *opaque, hwaddr addr, unsigned int size)
{
    //Esp32DportState *s = ESP32_DPORT(opaque);
    uint64_t r = 0;

    switch (addr) {
        case 0x8c:
            printf("read INTERRUPT_PRO_I2S0_INT_MAP\n");
            r=0x10;
            break; 

 

       //RTC_CNTL_RESET_STATE_REG


      default:
         printf("dport_read  %08X\n",(unsigned int)addr);
        break;

    }






    /*

    switch (addr) {
    case A_DPORT_APPCPU_RESET:
        r = s->appcpu_reset_state;
        break;
    case A_DPORT_APPCPU_CLK:
        r = s->appcpu_clkgate_state;
        break;
    case A_DPORT_APPCPU_RUNSTALL:
        r = s->appcpu_stall_state;
        break;
    case A_DPORT_APPCPU_BOOT_ADDR:
        r = s->appcpu_boot_addr;
        break;
    case A_DPORT_CPU_PER_CONF:
        r = s->cpuperiod_sel;
        break;
    case A_DPORT_PRO_CACHE_CTRL:
        r = s->cache_state[0].cache_ctrl_reg;
        break;
    case A_DPORT_PRO_CACHE_CTRL1:
        r = s->cache_state[0].cache_ctrl1_reg;
        break;
    case A_DPORT_APP_CACHE_CTRL:
        r = s->cache_state[1].cache_ctrl_reg;
        break;
    case A_DPORT_APP_CACHE_CTRL1:
        r = s->cache_state[1].cache_ctrl1_reg;
        break;
    case A_DPORT_PRO_DCACHE_DBUG0:
    case A_DPORT_APP_DCACHE_DBUG0:
        // in idle state 
        r = FIELD_DP32(0, DPORT_PRO_DCACHE_DBUG0, CACHE_STATE, 1);
        break;
    case PRO_DROM0_MMU_FIRST ... PRO_DROM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[0].drom0, PRO_DROM0_MMU_FIRST, addr);
        break;
    case PRO_IRAM0_MMU_FIRST ... PRO_IRAM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[0].iram0, PRO_IRAM0_MMU_FIRST, addr);
        break;
    case APP_DROM0_MMU_FIRST ... APP_DROM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[1].drom0, APP_DROM0_MMU_FIRST, addr);
        break;
    case APP_IRAM0_MMU_FIRST ... APP_IRAM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[1].iram0, APP_IRAM0_MMU_FIRST, addr);
        break;
    }
    */

    return r;
}

static void esp32s2_dport_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{

    switch (addr) {
        case 0x8c:
            printf("write INTERRUPT_PRO_I2S0_INT_MAP\n");
            break;

      default:
         printf("dport_write  %08X\n",(unsigned int)addr);
        break;

    }

    /*
    Esp32DportState *s = ESP32_DPORT(opaque);
    bool old_state;
    uint32_t old_val;
    switch (addr) {
    case A_DPORT_APPCPU_RESET:
        old_state = s->appcpu_reset_state;
        s->appcpu_reset_state = value & 1;
        if (old_state && !s->appcpu_reset_state) {
            qemu_irq_pulse(s->appcpu_reset_req);
        }
        break;
    case A_DPORT_APPCPU_CLK:
        s->appcpu_clkgate_state = value & 1;
        qemu_set_irq(s->appcpu_stall_req, s->appcpu_stall_state || !s->appcpu_clkgate_state);
        break;
    case A_DPORT_APPCPU_RUNSTALL:
        s->appcpu_stall_state = value & 1;
        qemu_set_irq(s->appcpu_stall_req, s->appcpu_stall_state || !s->appcpu_clkgate_state);
        break;
    case A_DPORT_APPCPU_BOOT_ADDR:
        s->appcpu_boot_addr = value;
        break;
    case A_DPORT_CPU_PER_CONF:
        s->cpuperiod_sel = value & R_DPORT_CPU_PER_CONF_CPUPERIOD_SEL_MASK;
        qemu_irq_pulse(s->clk_update_req);
        break;
    case A_DPORT_PRO_CACHE_CTRL:
        if (FIELD_EX32(value, DPORT_PRO_CACHE_CTRL, CACHE_FLUSH_ENA)) {
            value |= R_DPORT_PRO_CACHE_CTRL_CACHE_FLUSH_DONE_MASK;
            value &= ~R_DPORT_PRO_CACHE_CTRL_CACHE_FLUSH_ENA_MASK;
            esp32_cache_data_sync(&s->cache_state[0].drom0);
            esp32_cache_data_sync(&s->cache_state[0].iram0);
        }
        old_val = s->cache_state[0].cache_ctrl_reg;
        s->cache_state[0].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32s2_cache_state_update(&s->cache_state[0]);
        }
        break;
    case A_DPORT_PRO_CACHE_CTRL1:
        old_val = s->cache_state[0].cache_ctrl1_reg;
        s->cache_state[0].cache_ctrl1_reg = value;
        if (value != old_val) {
            esp32s2_cache_state_update(&s->cache_state[0]);
        }
        break;
    case A_DPORT_APP_CACHE_CTRL:
        if (FIELD_EX32(value, DPORT_APP_CACHE_CTRL, CACHE_FLUSH_ENA)) {
            value |= R_DPORT_APP_CACHE_CTRL_CACHE_FLUSH_DONE_MASK;
            value &= ~R_DPORT_APP_CACHE_CTRL_CACHE_FLUSH_ENA_MASK;
            esp32_cache_data_sync(&s->cache_state[1].drom0);
            esp32_cache_data_sync(&s->cache_state[1].iram0);
        }
        old_val = s->cache_state[1].cache_ctrl_reg;
        s->cache_state[1].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32s2_cache_state_update(&s->cache_state[1]);
        }
        break;
    case A_DPORT_APP_CACHE_CTRL1:
        old_val = s->cache_state[1].cache_ctrl1_reg;
        s->cache_state[1].cache_ctrl1_reg = value;
        if (value != old_val) {
            esp32s2_cache_state_update(&s->cache_state[1]);
        }
        break;
    case PRO_DROM0_MMU_FIRST ... PRO_DROM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[0].drom0, PRO_DROM0_MMU_FIRST, addr, value);
        break;
    case PRO_IRAM0_MMU_FIRST ... PRO_IRAM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[0].iram0, PRO_IRAM0_MMU_FIRST, addr, value);
        break;
    case APP_DROM0_MMU_FIRST ... APP_DROM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[1].drom0, APP_DROM0_MMU_FIRST, addr, value);
        break;
    case APP_IRAM0_MMU_FIRST ... APP_IRAM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[1].iram0, APP_IRAM0_MMU_FIRST, addr, value);
        break;        
    }
    */
}

static const MemoryRegionOps esp32_dport_ops = {
    .read =  esp32s2_dport_read,
    .write = esp32s2_dport_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

#if 0
static void esp32s2_cache_data_sync(Esp32CacheRegionState* crs)
{
    uint8_t* cache_data = (uint8_t*) memory_region_get_ram_ptr(&crs->mem);
    int n = 0;
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        uint32_t* cache_page = (uint32_t*) (cache_data + i * ESP32_CACHE_PAGE_SIZE);
        uint32_t mmu_entry = crs->mmu_table[i];
        if (!(mmu_entry & ESP32_CACHE_MMU_ENTRY_CHANGED)) {
            continue;
        }
        mmu_entry &= MMU_ENTRY_MASK;
        if (mmu_entry & ESP32_CACHE_MMU_INVALID_VAL) {
            uint32_t fill_val = crs->type == ESP32_DCACHE ? 0xbaadbaad : 0x00000000;
            for (int word = 0; word < ESP32_CACHE_PAGE_SIZE / sizeof(uint32_t); ++word) {
                cache_page[word] = fill_val;
            }
        } else {
            uint32_t phys_addr = mmu_entry * ESP32_CACHE_PAGE_SIZE;
            blk_pread(crs->cache->dport->flash_blk, phys_addr, cache_page, ESP32_CACHE_PAGE_SIZE);
        }
        crs->mmu_table[i] &= ~ESP32_CACHE_MMU_ENTRY_CHANGED;
        n++;
    }
    memory_region_flush_rom_device(&crs->mem, 0, ESP32_CACHE_REGION_SIZE);
}

static void esp32s2_cache_state_update(Esp32CacheState* cs)
{
    bool cache_enabled = FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL, CACHE_ENA) != 0;

    bool drom0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_DROM0) == 0;
    if (!cs->drom0.mem.enabled && drom0_enabled) {
        esp32_cache_data_sync(&cs->drom0);
    }
    memory_region_set_enabled(&cs->drom0.mem, drom0_enabled);

    bool iram0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_IRAM0) == 0;
    if (!cs->iram0.mem.enabled && iram0_enabled) {
        esp32_cache_data_sync(&cs->iram0);
    }
    memory_region_set_enabled(&cs->iram0.mem, iram0_enabled);
}
#endif

static void esp32s2_cache_region_reset(Esp32CacheRegionState *crs)
{
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        crs->mmu_table[i] = ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
}

static void esp32s2_cache_reset(Esp32CacheState *cs)
{
    esp32s2_cache_region_reset(&cs->drom0);
    esp32s2_cache_region_reset(&cs->iram0);
}
static const MemoryRegionOps esp32_cache_ops = {
    .write = NULL,
    .endianness = DEVICE_LITTLE_ENDIAN,
};


static void esp32s2_dport_reset(DeviceState *dev)
{
    Esp32DportState *s = ESP32_DPORT(dev);

    s->appcpu_boot_addr = 0;
    s->appcpu_clkgate_state = false;
    s->appcpu_reset_state = true;
    s->appcpu_stall_state = false;
    esp32s2_cache_reset(&s->cache_state[0]);
    esp32s2_cache_reset(&s->cache_state[1]);
}

static void esp32s2_dport_realize(DeviceState *dev, Error **errp)
{
    Esp32DportState *s = ESP32_DPORT(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    s->cpu_count = ms->smp.cpus;

    for (int i = 0; i < s->cpu_count; ++i) {
        char name[16];
        snprintf(name, sizeof(name), "cpu%d", i);
        object_property_set_link(OBJECT(&s->intmatrix), OBJECT(qemu_get_cpu(i)), name, &error_abort);
    }
    object_property_set_bool(OBJECT(&s->intmatrix), true, "realized", &error_abort);

    //object_property_set_bool(OBJECT(&s->crosscore_int), true, "realized", &error_abort);

    //for (int index = 0; index < ESP32_DPORT_CROSSCORE_INT_COUNT; ++index) {
    //    qemu_irq target = qdev_get_gpio_in(DEVICE(&s->intmatrix), ETS_FROM_CPU_INTR0_SOURCE + index);
    //    assert(target);
    //    sysbus_connect_irq(SYS_BUS_DEVICE(&s->crosscore_int), index, target);
    //}
}

static void esp32_cache_init_region(Esp32CacheState *cs,
                                    Esp32CacheRegionState *crs,
                                    Esp32CacheRegionType type,
                                    uint32_t enable_mask,
                                    const char* name, hwaddr base)
{
    char desc[16];
    crs->cache = cs;
    crs->type = type;
    crs->base = base;
    snprintf(desc, sizeof(desc), "cpu%d-%s", cs->core_id, name);
    memory_region_init_rom_device(&crs->mem, OBJECT(cs->dport),
                                  &esp32_cache_ops, crs,
                                  desc, ESP32_CACHE_REGION_SIZE, &error_abort);
}

static void esp32s2_dport_init(Object *obj)
{
    Esp32DportState *s = ESP32_DPORT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_dport_ops, s,
                          TYPE_ESP32_DPORT, 0x1000 /* DR_REG_DMA_COPY_BASE-DR_REG_SENSITIVE_BASE*/);
    sysbus_init_mmio(sbd, &s->iomem);

/*
perif
0x600000000x600B_FFFF

0x618000000x6180_3FFF
*/




    object_initialize_child(obj, "intmatrix", &s->intmatrix, sizeof(s->intmatrix), TYPE_ESP32_INTMATRIX, &error_abort, NULL);
    memory_region_add_subregion_overlap(&s->iomem, DR_REG_INTERRUPT_BASE, &s->intmatrix.iomem, -1);

    //object_initialize_child(obj, "crosscore_int", &s->crosscore_int, sizeof(s->crosscore_int), TYPE_ESP32_CROSSCORE_INT,
    //                        &error_abort, NULL);
    //memory_region_add_subregion_overlap(&s->iomem, ESP32_DPORT_CROSSCORE_INT_BASE, &s->crosscore_int.iomem, -1);


    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        Esp32CacheState* cs = &s->cache_state[i];
        cs->core_id = i;
        cs->dport = s;
        esp32_cache_init_region(cs, &cs->drom0, ESP32_DCACHE,
                                R_DPORT_PRO_CACHE_CTRL1_MASK_DROM0_MASK, "drom0", 0x3F400000);
        esp32_cache_init_region(cs, &cs->iram0, ESP32_ICACHE,
                                R_DPORT_PRO_CACHE_CTRL1_MASK_IRAM0_MASK, "iram0", 0x40000000);
    }

    qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_stall_req, ESP32_DPORT_APPCPU_STALL_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_reset_req, ESP32_DPORT_APPCPU_RESET_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->clk_update_req, ESP32_DPORT_CLK_UPDATE_GPIO, 1);
}

static Property esp32s2_dport_properties[] = {
    DEFINE_PROP_DRIVE("flash", Esp32DportState, flash_blk),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s2_dport_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32s2_dport_reset;
    dc->realize = esp32s2_dport_realize;
    dc->props = esp32s2_dport_properties;
}

static const TypeInfo esp32s2_dport_info = {
    .name = TYPE_ESP32_DPORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32DportState),
    .instance_init = esp32s2_dport_init,
    .class_init = esp32s2_dport_class_init
};

static void esp32s2_dport_register_types(void)
{
    type_register_static(&esp32s2_dport_info);
}

type_init(esp32s2_dport_register_types)
