/*
 * Bosh BME280 temperature sensor.
 *
 * Copyright (C) 2008 Nokia Corporation
 * Written by Andrzej Zaborowski <andrew@openedhand.com>
 * Copyright (C) 2017 Olof Astrand
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.

This module will currently just allow to read from a register dump of the actual hardware.
This is done in order to understand how qemu and the esp32 i2c controller interacts 

Displaying all regs
0x08 0802 0500 0809 0208 0a07 020d 0f06 0006 0f00 060e 0705 0608 0302 0000 0904 080f 
0x09 0606 0d06 0d00 000b 0a02 0201 0e09 0f0f 0f09 0f0f 0a0c 0206 000a 0d08 0b0d 0100 
0x0a 0000 040b 0a05 0000 0000 0000 0000 0000 0000 0000 0000 0000 0303 0000 0000 0c00 
0x0b 0000 0504 0000 0000 0000 0000 0600 0002 0000 0001 0f0f 0f0f 010f 0600 0003 0000 
0x0c 0000 0000 0204 0f0f 0000 0000 0000 0000 0001 0000 0000 0000 0000 0000 0000 0000 
0x0d 0600 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 0000 
0x0e 0000 0609 0001 0000 0104 0002 0000 010e 030c 0401 0f0f 0f0f 0f0f 0f0f 0f0f 0f0f 
0x0f 0f0f 0000 0001 0000 0204 0000 0000 040c 0d01 0000 070f 0f05 0000 0606 0705 0800 

 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/i2c/i2c.h"
#include "tmpbme280.h"
#include "qapi/error.h"
#include "qapi/visitor.h"


// Firs register = 0x80
unsigned short int reg_dump[]= 
{
    0x0802, 0x0500, 0x0809, 0x0208, 0x0a07, 0x020d, 0x0f06, 0x0006, 0x0f00, 0x060e, 0x0705, 0x0608, 0x0302, 0x0000, 0x0904, 0x080f, 
    0x0606, 0x0d06, 0x0d00, 0x000b, 0x0a02, 0x0201, 0x0e09, 0x0f0f, 0x0f09, 0x0f0f, 0x0a0c, 0x0206, 0x000a, 0x0d08, 0x0b0d, 0x0100, 
    0x0000, 0x040b, 0x0a05, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0303, 0x0000, 0x0000, 0x0c00, 
    0x0000, 0x0504, 0x0000, 0x0000, 0x0000, 0x0000, 0x0600, 0x0002, 0x0000, 0x0001, 0x0f0f, 0x0f0f, 0x010f, 0x0600, 0x0003, 0x0000, 
    0x0000, 0x0000, 0x0204, 0x0f0f, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    0x0600, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 
    0x0000, 0x0609, 0x0001, 0x0000, 0x0104, 0x0002, 0x0000, 0x010e, 0x030c, 0x0401, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 0x0f0f, 
    0x0f0f, 0x0000, 0x0001, 0x0000, 0x0204, 0x0000, 0x0000, 0x040c, 0x0d01, 0x0000, 0x070f, 0x0f05, 0x0000, 0x0606, 0x0705, 0x0800
};

static void tmpbme280_get_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMPBME280State *s = TMPBME280(obj);
    int64_t value = s->temperature * 1000 / 256;

    visit_type_int(v, name, &value, errp);
}

/* Units are 0.001 centigrades relative to 0 C.  s->temperature is 8.8
 * fixed point, so units are 1/256 centigrades.  A simple ratio will do.
 */
static void tmpbme280_set_temperature(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    TMPBME280State *s = TMPBME280(obj);
    Error *local_err = NULL;
    int64_t temp;

    visit_type_int(v, name, &temp, &local_err);
    if (local_err) {
        error_propagate(errp, local_err);
        return;
    }
    if (temp >= 128000 || temp < -128000) {
        error_setg(errp, "value %" PRId64 ".%03" PRIu64 " °C is out of range",
                   temp / 1000, temp % 1000);
        return;
    }

    s->temperature = (int16_t) (temp * 256 / 1000);

    //tmpbme280_alarm_update(s);
}


static void tmpbme280_read(TMPBME280State *s)
{
    s->len = 0;

    int regnum=s->pointer & 0xff;
    int reg_pos=regnum-0x80;


    s->buf[s->len ++] = ((uint16_t) reg_dump[reg_pos]) >> 8;
    s->buf[s->len ++] = ((uint16_t) reg_dump[reg_pos]) >> 0;

    // Cache all remaining registers
    while(reg_pos< 16*8) {
        reg_pos++;

        s->buf[s->len ++] = ((uint16_t) reg_dump[reg_pos]) >> 8;
        s->buf[s->len ++] = ((uint16_t) reg_dump[reg_pos]) >> 0;
    }

    // Reset to start of buffer
    s->len = 0;

}

static void tmpbme280_write(TMPBME280State *s)
{
    switch (s->pointer & 3) {
    case TMPBME280_REG_TEMPERATURE:
        break;

    case TMPBME280_REG_CONFIG:
        if (s->buf[0] & ~s->config & (1 << 0))			/* SD */
            printf("%s: TMPBME280 shutdown\n", __FUNCTION__);
        s->config = s->buf[0];
        break;

    case TMPBME280_REG_T_LOW:
    case TMPBME280_REG_T_HIGH:
        if (s->len >= 3)
            s->limit[s->pointer & 1] = (int16_t)
                    ((((uint16_t) s->buf[0]) << 8) | s->buf[1]);
        //tmpbme280_alarm_update(s);
        break;
    }
}

// Called when master reads from slave, 
static int tmpbme280_rx(I2CSlave *i2c)
{
    TMPBME280State *s = TMPBME280(i2c);
    fprintf(stderr,"tmpbme280_rx %d\n",s->buf[s->len]);

    if (s->len < 2) {
        return s->buf[s->len ++];
    } else {
        return 0xff;
    }
}

// Called when master writes to slave, should only be register number (pointer)
static int tmpbme280_tx(I2CSlave *i2c, uint8_t data)
{
    TMPBME280State *s = TMPBME280(i2c);

    fprintf(stderr,"tmpbme280_tx %d\n",data);

    if (s->len == 0) {
        s->pointer = data;
        s->len++;
    } else {
        if (s->len <= 2) {
            s->buf[s->len - 1] = data;
        }
        s->len++;
        tmpbme280_write(s);
    }

    return 0;
}

static void tmpbme280_event(I2CSlave *i2c, enum i2c_event event)
{
    TMPBME280State *s = TMPBME280(i2c);

    switch (event) {
        case I2C_START_RECV:
             fprintf(stderr,"tmpbme280_event I2C_START_RECV\n");
          break;
          case I2C_START_SEND:
             fprintf(stderr,"tmpbme280_event I2C_START_SEND\n");
          break;
          case I2C_FINISH:
             fprintf(stderr,"tmpbme280_event I2C_FINISH\n");
          break;
           case I2C_NACK:
             fprintf(stderr,"tmpbme280_event I2C_NACK\n");
          break;

          break;
          default:
            fprintf(stderr,"tmpbme280_event %d\n",event);
          break;          
    }

    if (event == I2C_START_SEND) {

    }

    if (event == I2C_START_RECV) {
        // prepare buffer with data from what pointer
        tmpbme280_read(s);
    }

    s->len = 0;
}

static int tmpbme280_post_load(void *opaque, int version_id)
{
    TMPBME280State *s = opaque;


    return 0;
}

static const VMStateDescription vmstate_tmpbme280 = {
    .name = "TMPBME280",
    .version_id = 0,
    .minimum_version_id = 0,
    .post_load = tmpbme280_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(len, TMPBME280State),
        VMSTATE_UINT8_ARRAY(buf, TMPBME280State, 2),
        VMSTATE_UINT8(pointer, TMPBME280State),
        VMSTATE_UINT8(config, TMPBME280State),
        VMSTATE_INT16(temperature, TMPBME280State),
        VMSTATE_INT16_ARRAY(limit, TMPBME280State, 2),
        VMSTATE_I2C_SLAVE(i2c, TMPBME280State),
        VMSTATE_END_OF_LIST()
    }
};

static void tmpbme280_reset(I2CSlave *i2c)
{
    TMPBME280State *s = TMPBME280(i2c);

    s->temperature = 0;
    s->pointer = 0;
    s->config = 0;

}

static int tmpbme280_init(I2CSlave *i2c)
{
    TMPBME280State *s = TMPBME280(i2c);

    qdev_init_gpio_out(&i2c->qdev, &s->pin, 1);

    tmpbme280_reset(&s->i2c);

    return 0;
}

static void tmpbme280_initfn(Object *obj)
{
    object_property_add(obj, "temperature", "int",
                        tmpbme280_get_temperature,
                        tmpbme280_set_temperature, NULL, NULL, NULL);
}

static void tmpbme280_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = tmpbme280_init;
    k->event = tmpbme280_event;
    k->recv = tmpbme280_rx;
    k->send = tmpbme280_tx;
    dc->vmsd = &vmstate_tmpbme280;
}

static const TypeInfo tmpbme280_info = {
    .name          = TYPE_TMPBME280,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TMPBME280State),
    .instance_init = tmpbme280_initfn,
    .class_init    = tmpbme280_class_init,
};

static void tmpbme280_register_types(void)
{
    type_register_static(&tmpbme280_info);
}

type_init(tmpbme280_register_types)
