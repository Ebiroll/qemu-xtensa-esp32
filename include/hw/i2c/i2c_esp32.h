
#ifndef QEMU_I2CESP32_H
#define QEMU_I2CESP32_H

#include "hw/qdev.h"

/* The QEMU I2C implementation only supports simple transfers that complete
   immediately.  It does not support slave devices that need to be able to
   defer their response (eg. CPU slave interfaces where the data is supplied
   by the device driver in response to an interrupt).  */

#define TYPE_ESP32_I2C "esp32_i2c"


void esp32_i2c_fifo_dataSet(int offset,unsigned int data);

void esp32_i2c_interruptSet(qemu_irq new_irq);



#endif