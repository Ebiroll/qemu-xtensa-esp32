/*
 * Bosh BME280 temperature sensor.
 *
 * Browse the data sheet:
 *
 *    https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
 *
 * Copyright (C) 2012 Alex Horn <alex.horn@cs.ox.ac.uk>
 * Copyright (C) 2008-2012 Andrzej Zaborowski <balrogg@gmail.com>
 * Copyright (C) 2017 Olof Astrand <olof.astrand@gmail.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or
 * later. See the COPYING file in the top-level directory.
 */
#ifndef QEMU_TMPBME280_H
#define QEMU_TMPBME280_H

typedef enum TMPBME280Reg {
    TMPBME280_REG_TEMPERATURE = 0,
    TMPBME280_REG_CONFIG,
    TMPBME280_REG_T_LOW,
    TMPBME280_REG_T_HIGH,
} TMPBME280Reg;


#include "hw/i2c/i2c.h"

#define TYPE_TMPBME280 "tmpbme280"
#define TMPBME280(obj) OBJECT_CHECK(TMPBME280State, (obj), TYPE_TMPBME280)

/**
 * TMPBME280State:
 * @config: Bits 5 and 6 (value 32 and 64) determine the precision of the
 * temperature. See Table 8 in the data sheet.
 *
 * @see_also: http://www.ti.com/lit/gpn/tmpbme280
 */
typedef struct TMPBME280State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t buf[2];
    qemu_irq pin;

    uint8_t pointer;
    uint8_t config;
    int16_t temperature;
    int16_t limit[2];
    int faults;
    uint8_t alarm;
} TMPBME280State;

#endif
