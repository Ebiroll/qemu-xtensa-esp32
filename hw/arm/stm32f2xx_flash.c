/*-
 * Copyright (c) 2013
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <stdio.h>
#include <inttypes.h>
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "sysemu/blockdev.h"
#include "hw/hw.h"
#include "hw/block/flash.h"
#include "block/block.h"
#include "sysemu/block-backend.h"
#include "hw/sysbus.h"


struct f2xx_flash {
    SysBusDevice busdev;
    BlockBackend *blk;
    hwaddr base_address;
    uint32_t size;

    MemoryRegion mem;
    void *data;
}; // f2xx_flash_t;

/* */
f2xx_flash_t *f2xx_flash_register(BlockBackend *blk, hwaddr base,
                                  hwaddr size)
{
    DeviceState *dev = qdev_create(NULL, "f2xx.flash");
    //SysBusDevice *busdev = SYS_BUS_DEVICE(dev);
    f2xx_flash_t *flash = (f2xx_flash_t *)object_dynamic_cast(OBJECT(dev),
                                                              "f2xx.flash");
    qdev_prop_set_uint32(dev, "size", size);
    qdev_prop_set_uint64(dev, "base_address", base);
    if (blk) {
    	Error *err = NULL;
        qdev_prop_set_drive(dev, "drive", blk, &err);
        if (err) {
            printf("%s, have no drive???\n", __func__);
            return NULL;
        }
    }
    qdev_init_nofail(dev);
    //sysbus_mmio_map(busdev, 0, base);
    return flash;
}

/* */

static uint64_t
f2xx_flash_read(void *arg, hwaddr offset, unsigned int size)
{
    //f2xx_flash_t *flash = arg;

    printf("read offset 0x%jx size %ju\n", (uintmax_t)offset, (uintmax_t)size);
    return 0;
}

static void
f2xx_flash_write(void *arg, hwaddr offset, uint64_t data, unsigned int size)
{
}

static const MemoryRegionOps f2xx_flash_ops = {
    .read = f2xx_flash_read,
    .write = f2xx_flash_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

MemoryRegion *get_system_memory(void); /* XXX */

static int f2xx_flash_init(SysBusDevice *dev)
{
    Error *err = NULL;
    f2xx_flash_t *flash = FROM_SYSBUS(typeof(*flash), dev);

//    memory_region_init_rom_device(&flash->mem, &f2xx_flash_ops, flash, "name",
//      size);
    memory_region_init_ram(&flash->mem, NULL, "f2xx.flash", flash->size, &err);

    vmstate_register_ram(&flash->mem, DEVICE(flash));
    //vmstate_register_ram_global(&flash->mem);
    memory_region_set_readonly(&flash->mem, true);
    memory_region_add_subregion(get_system_memory(), flash->base_address, &flash->mem);
//    sysbus_init_mmio(dev, &flash->mem);

    flash->data = memory_region_get_ram_ptr(&flash->mem);
    memset(flash->data, 0xff, flash->size);
    if (flash->blk) {
        int r;
        r = blk_pread(flash->blk, 0<< BDRV_SECTOR_BITS, flash->data, blk_getlength(flash->blk)/BDRV_SECTOR_SIZE  << BDRV_SECTOR_BITS);
        if (r < 0) {
            vmstate_unregister_ram(&flash->mem, DEVICE(flash));
            // memory_region_destroy(&flash->mem);
            return 1;
        }
    }

    return 0;
}

static Property f2xx_flash_properties[] = {
    DEFINE_PROP_DRIVE("drive", struct f2xx_flash, blk),
    DEFINE_PROP_UINT32("size", struct f2xx_flash, size, 512*1024),
    DEFINE_PROP_UINT64("base_address", struct f2xx_flash, base_address, 0x08000000),
    DEFINE_PROP_END_OF_LIST(),
};

static void f2xx_flash_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = f2xx_flash_init;
    dc->props = f2xx_flash_properties;
}

static const TypeInfo f2xx_flash_info = {
    .name = "f2xx.flash",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct f2xx_flash),
    .class_init = f2xx_flash_class_init,
};

static void f2xx_flash_register_types(void)
{
    type_register_static(&f2xx_flash_info);
}

type_init(f2xx_flash_register_types)
