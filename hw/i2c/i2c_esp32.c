/*
 * ESP32 I2C controller
 *
 * Copyright (c) 2017 Olof Astrand
 *
 * This file is derived from i2c/versatile_i2c.c by CodeSourcery & 
 *    Oskar Andero <oskar.andero@gmail.com>
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "bitbang_i2c.h"
#include "qemu/log.h"

#define I2C_COMD0_REG          (0x0058)
#define I2C_FIFO_CONF_REG          (0x0018)


#define TYPE_ESP32_I2C "esp32_i2c"
#define ESP32_I2C(obj) \
    OBJECT_CHECK(Esp32I2CState, (obj), TYPE_ESP32_I2C)

typedef struct Esp32I2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    bitbang_i2c_interface *bitbang;
    int out;
    int in;
} Esp32I2CState;

static uint64_t esp32_i2c_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    Esp32I2CState *s = (Esp32I2CState *)opaque;

    if (offset == 0) {
        return (s->out & 1) | (s->in << 1);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%x\n", __func__, (int)offset);
        return -1;
    }
}

static void esp32_i2c_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    Esp32I2CState *s = (Esp32I2CState *)opaque;

    switch (offset) {
    case 0:
        s->out |= value & 3;
        break;
    case 4:
        s->out &= ~value;
        break;

   case I2C_FIFO_CONF_REG:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: I2C_COMD0_REG 0x%x\n", __func__, (int)value);
         break;

    case I2C_COMD0_REG:
            qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: I2C_COMD0_REG 0x%x\n", __func__, (int)value);
         break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%x\n", __func__, (int)offset);
    }
    bitbang_i2c_set(s->bitbang, BITBANG_I2C_SCL, (s->out & 1) != 0);
    s->in = bitbang_i2c_set(s->bitbang, BITBANG_I2C_SDA, (s->out & 2) != 0);
}

static const MemoryRegionOps esp32_i2c_ops = {
    .read = esp32_i2c_read,
    .write = esp32_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void esp32_i2c_init(Object *obj)
{
    DeviceState *dev = DEVICE(obj);
    Esp32I2CState *s = ESP32_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    I2CBus *bus;

    bus = i2c_init_bus(dev, "i2c");
    s->bitbang = bitbang_i2c_init(bus);
    memory_region_init_io(&s->iomem, obj, &esp32_i2c_ops, s,
                          "esp32_i2c", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static const TypeInfo esp32_i2c_info = {
    .name          = TYPE_ESP32_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2CState),
    .instance_init = esp32_i2c_init,
};

static void esp32_i2c_register_types(void)
{
    type_register_static(&esp32_i2c_info);
}

type_init(esp32_i2c_register_types)
