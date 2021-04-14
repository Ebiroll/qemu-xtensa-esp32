/*
 * Nordic Semiconductor nRF52 SoC
 * 
 * 
 * https://www.nordicsemi.com/-/media/DocLib/Other/Product_Spec/nRF52832PSv14.pdf* 
 *  http://infocenter.nordicsemi.com/pdf/nRF51_RM_v3.0.1.pdf
 * 
 *  https://en.wikipedia.org/wiki/ARM_Cortex-M#Cortex-M4
 *
 * https://devzone.nordicsemi.com/b/blog/posts/new-features-in-nrf52
 *
 * Copyright 2018 Joel Stanley <joel@jms.id.au>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/arm/arm.h"
#include "hw/sysbus.h"
#include "hw/boards.h"
#include "hw/devices.h"
#include "hw/misc/unimp.h"
#include "exec/address-spaces.h"
#include "sysemu/sysemu.h"
#include "qemu/log.h"
#include "cpu.h"

#include "hw/arm/nrf52.h"
#include "hw/arm/nrf52_soc.h"

/*
 * The size and base is for the NRF52832 part. If other parts
 * are supported in the future, add a sub-class of NRF52SoC for
 * the specific variants
 */
#define NRF52832_FLASH_PAGES    8*256
#define NRF52832_SRAM_PAGES     2*32
#define NRF52832_FLASH_SIZE     (NRF52832_FLASH_PAGES * NRF52_PAGE_SIZE)
#define NRF52832_SRAM_SIZE      (NRF52832_SRAM_PAGES * NRF52_PAGE_SIZE)

#define BASE_TO_IRQ(base) ((base >> 12) & 0x1F)

static uint64_t clock_read(void *opaque, hwaddr addr, unsigned int size)
{
    qemu_log_mask(LOG_UNIMP, "%s: 0x%" HWADDR_PRIx " [%u]\n",
                  __func__, addr, size);
    return 1;
}

static void clock_write(void *opaque, hwaddr addr, uint64_t data,
                        unsigned int size)
{
    qemu_log_mask(LOG_UNIMP, "%s: 0x%" HWADDR_PRIx " <- 0x%" PRIx64 " [%u]\n",
                  __func__, addr, data, size);
}

static const MemoryRegionOps clock_ops = {
    .read = clock_read,
    .write = clock_write
};


static void nrf52_soc_realize(DeviceState *dev_soc, Error **errp)
{
    NRF52State *s = NRF52_SOC(dev_soc);
    MemoryRegion *mr;
    Error *err = NULL;
    uint8_t i = 0;
    hwaddr base_addr = 0;

    //s->cpu


    printf("REALIZE!!!\n\n");


    if (!s->board_memory) {
        error_setg(errp, "memory property was not set");
        return;
    }

    object_property_set_link(OBJECT(&s->cpu), OBJECT(&s->container), "memory",
            &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    object_property_set_bool(OBJECT(&s->cpu), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    memory_region_add_subregion_overlap(&s->container, 0, s->board_memory, -1);

    memory_region_init_ram(&s->sram, OBJECT(s), "nrf52.sram", s->sram_size,
                           &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    memory_region_add_subregion(&s->container, NRF52_SRAM_BASE, &s->sram);

    /* UART */
    object_property_set_bool(OBJECT(&s->uart), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->uart), 0);
    memory_region_add_subregion_overlap(&s->container, NRF52_UART_BASE, mr, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu),
                       BASE_TO_IRQ(NRF52_UART_BASE)));

    /* RNG */
    object_property_set_bool(OBJECT(&s->rng), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->rng), 0);
    memory_region_add_subregion_overlap(&s->container, NRF52_RNG_BASE, mr, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rng), 0,
                       qdev_get_gpio_in(DEVICE(&s->cpu),
                       BASE_TO_IRQ(NRF52_RNG_BASE)));

    /* UICR, FICR, NVMC, FLASH */
    object_property_set_uint(OBJECT(&s->nvm), s->flash_size, "flash-size",
                             &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    object_property_set_bool(OBJECT(&s->nvm), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 0);
    memory_region_add_subregion_overlap(&s->container, NRF52_NVMC_BASE, mr, 0);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 1);
    memory_region_add_subregion_overlap(&s->container, NRF52_FICR_BASE, mr, 0);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 2);
    memory_region_add_subregion_overlap(&s->container, NRF52_UICR_BASE, mr, 0);
    MemoryRegion *ram;
        ram = g_malloc(sizeof(*ram));
    memset(ram,0,sizeof(*ram));
    MemoryRegion *system_memory = get_system_memory();

    // memory_region_init_ram(ram, NULL, "iram0", 0x1fffffff,  // 00000
    //                       &error_abort);
    //memory_region_init_ram(ram, NULL, "iram0", 0x20000000,  // 00000
    //                       &error_abort);

    memory_region_init_ram_nomigrate(ram, NULL, "nrf52.fakeram", 0x20000000,  // 00000
                           &error_abort);

    memory_region_add_subregion(system_memory, 0x0, ram);


    //mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 3);
    //memory_region_add_subregion_overlap(&s->container, NRF52_FLASH_BASE, mr, 0);

    /* GPIO */
    object_property_set_bool(OBJECT(&s->gpio), true, "realized", &err);
    if (err) {
        error_propagate(errp, err);
        return;
    }

    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->gpio), 0);
    memory_region_add_subregion_overlap(&s->container, NRF52_GPIO_BASE, mr, 0);

    /* Pass all GPIOs to the SOC layer so they are available to the board */
    qdev_pass_gpios(DEVICE(&s->gpio), dev_soc, NULL);

    /* TIMER */
    for (i = 0; i < NRF52_NUM_TIMERS; i++) {
        object_property_set_bool(OBJECT(&s->timer[i]), true, "realized", &err);
        if (err) {
            error_propagate(errp, err);
            return;
        }

        base_addr = NRF52_TIMER_BASE + i * NRF52_TIMER_SIZE;

        sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer[i]), 0, base_addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->cpu),
                                            BASE_TO_IRQ(base_addr)));
    }

    /* STUB Peripherals */
    memory_region_init_io(&s->clock, NULL, &clock_ops, NULL,
                          "NRF52_soc.clock", 0x1000);
    memory_region_add_subregion_overlap(&s->container,
                                        NRF52_IOMEM_BASE, &s->clock, -1);

    create_unimplemented_device("NRF52_soc.io", NRF52_IOMEM_BASE,
                                NRF52_IOMEM_SIZE);
    create_unimplemented_device("NRF52_soc.private",
                                NRF52_PRIVATE_BASE, NRF52_PRIVATE_SIZE);
}

static void nrf52_soc_init(Object *obj)
{
    uint8_t i = 0;

    NRF52State *s = NRF52_SOC(obj);

    printf("SOC INIT!!!\n\n");


    memory_region_init(&s->container, obj, "nrf52-container", UINT64_MAX);

    sysbus_init_child_obj(OBJECT(s), "armv7m4", OBJECT(&s->cpu), sizeof(s->cpu),
                          TYPE_ARMV7M);


   object_property_set_bool(OBJECT(&s->cpu), false,
                                     "start-powered-off", &error_abort);

    qdev_prop_set_string(DEVICE(&s->cpu), "cpu-type",
                         ARM_CPU_TYPE_NAME("cortex-m3"));
    qdev_prop_set_uint32(DEVICE(&s->cpu), "num-irq", 32);

    sysbus_init_child_obj(obj, "uart", &s->uart, sizeof(s->uart),
                           TYPE_NRF51_UART);
    object_property_add_alias(obj, "serial0", OBJECT(&s->uart), "chardev",
                              &error_abort);

    sysbus_init_child_obj(obj, "rng", &s->rng, sizeof(s->rng),
                           TYPE_NRF51_RNG);

    sysbus_init_child_obj(obj, "nvm", &s->nvm, sizeof(s->nvm), TYPE_NRF51_NVM);

    sysbus_init_child_obj(obj, "gpio", &s->gpio, sizeof(s->gpio),
                          TYPE_NRF51_GPIO);

    for (i = 0; i < NRF52_NUM_TIMERS; i++) {
        sysbus_init_child_obj(obj, "timer[*]", &s->timer[i],
                              sizeof(s->timer[i]), TYPE_NRF51_TIMER);

    }
}

static Property nrf52_soc_properties[] = {
    DEFINE_PROP_LINK("memory", NRF52State, board_memory, TYPE_MEMORY_REGION,
                     MemoryRegion *),
    DEFINE_PROP_UINT32("sram-size", NRF52State, sram_size, NRF52832_SRAM_SIZE),
    DEFINE_PROP_UINT32("flash-size", NRF52State, flash_size,
                       NRF52832_FLASH_SIZE),
    DEFINE_PROP_END_OF_LIST(),
};

static void nrf52_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = nrf52_soc_realize;
    dc->props = nrf52_soc_properties;
}

static const TypeInfo nrf52_soc_info = {
    .name          = TYPE_NRF52_SOC,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NRF52State),
    .instance_init = nrf52_soc_init,
    .class_init    = nrf52_soc_class_init,
};

static void nrf52_soc_types(void)
{
    type_register_static(&nrf52_soc_info);
}
type_init(nrf52_soc_types)
