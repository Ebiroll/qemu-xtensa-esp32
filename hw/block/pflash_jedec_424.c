/*
 *  CFI parallel flash with Jedec (42.4) command set emulation
 *
 *  Copyright (c) 2006 Thorsten Zitterell
 *  Copyright (c) 2005 Jocelyn Mayer
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

/*
 * For now, this code can emulate flashes of 1, 2 or 4 bytes width.
 * Supported commands/modes are:
 * - flash read
 * - flash write
 * - flash ID read
 * - sector erase
 * - CFI queries
 *
 * It does not support timings
 * It does not support flash interleaving
 * It does not implement software data protection as found in many real chips
 * It does not implement erase suspend/resume commands
 * It does not implement multiple sectors erase
 *
 * It does not implement much more ...
 */
#include <stdio.h>
#include <inttypes.h>
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "hw/hw.h"
#include "hw/block/flash.h"
#include "block/block.h"
#include "block/block_int.h"
#include "sysemu/block-backend.h"
#include "qemu/bitops.h"
#include "exec/address-spaces.h"
#include "qemu/host-utils.h"
#include "hw/sysbus.h"

pflash_t *pflash_jedec_424_register(hwaddr base,
                                DeviceState *qdev, const char *name,
                                hwaddr size,
                                BlockBackend *blk,
                                uint32_t sector_len, int nb_blocs, uint32_t bank_size,
                                int bank_width, uint16_t id0, uint16_t id1,
                                uint16_t id2, uint16_t id3, int be);

MemoryRegion *pflash_jedec_424_get_memory(pflash_t *fl);


#define PFLASH_BUG(fmt, ...) \
do { \
    fprintf(stderr, "PFLASH: Possible BUG - " fmt, ## __VA_ARGS__); \
    exit(1); \
} while(0)

//#define PFLASH_DEBUG
#ifdef PFLASH_DEBUG
// NOTE: The usleep() helps the MacOS stdout from freezing when we have a lot of print out
#define DPRINTF(fmt, ...)                                   \
do {                                                        \
    printf("PFLASH: " fmt , ## __VA_ARGS__);                \
    usleep(1000);                                           \
} while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

#define TYPE_CFI_PFLASH_JEDEC_424 "cfi.pflash.jedec-42.4"
#define CFI_PFLASH_JEDEC(obj) OBJECT_CHECK(pflash_t, (obj), TYPE_CFI_PFLASH_JEDEC_424)

#define PFLASH_MAX_BANKS 8

struct pflash_t {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    BlockBackend *blk;
    uint32_t nb_blocs;
    uint64_t sector_len;
    uint8_t bank_width;
    uint8_t device_width; /* If 0, device width not specified. */
    uint8_t max_device_width;  /* max device width in bytes */
    uint32_t bank_size;  /* size of each bank */
    uint8_t be;
    int ro;
    uint8_t wcycle[PFLASH_MAX_BANKS];  // which write-cycle this bank is in
    uint8_t cmd[PFLASH_MAX_BANKS];  // which command we're processing
    uint8_t global_cmd;  // some operations don't have separate bank states
    uint8_t status;
    uint16_t ident0;
    uint16_t ident1;
    uint16_t ident2;
    uint16_t ident3;
    uint8_t cfi_len;
    uint8_t cfi_table[0x60];
    uint64_t counter;
    unsigned int writeblock_size;
    MemoryRegion mem;
    char *name;
    void *storage;
    uint16_t configuration_register;
};

static const VMStateDescription vmstate_pflash = {
    .name = "pflash_jedec_424",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(wcycle, pflash_t, PFLASH_MAX_BANKS),
        VMSTATE_UINT8_ARRAY(cmd, pflash_t, PFLASH_MAX_BANKS),
        VMSTATE_UINT8(global_cmd, pflash_t),
        VMSTATE_UINT8(status, pflash_t),
        VMSTATE_UINT64(counter, pflash_t),
        VMSTATE_END_OF_LIST()
    }
};

static void pflash_reset_state(struct pflash_t* pfl)
{
    memset(pfl->wcycle, 0, sizeof(pfl->wcycle));
    memset(pfl->cmd, 0, sizeof(pfl->cmd));
    pfl->global_cmd = 0;
}

/* Perform a CFI query based on the bank width of the flash.
 * If this code is called we know we have a device_width set for
 * this flash.
 */
static uint32_t pflash_cfi_query(pflash_t *pfl, hwaddr offset)
{
    int i;
    uint32_t resp = 0;
    hwaddr boff;

    /* Adjust incoming offset to match expected device-width
     * addressing. CFI query addresses are always specified in terms of
     * the maximum supported width of the device.  This means that x8
     * devices and x8/x16 devices in x8 mode behave differently.  For
     * devices that are not used at their max width, we will be
     * provided with addresses that use higher address bits than
     * expected (based on the max width), so we will shift them lower
     * so that they will match the addresses used when
     * device_width==max_device_width.
     */
    boff = offset >> (ctz32(pfl->bank_width) +
                      ctz32(pfl->max_device_width) - ctz32(pfl->device_width));

    if (boff > pfl->cfi_len) {
        return 0;
    }
    /* Now we will construct the CFI response generated by a single
     * device, then replicate that for all devices that make up the
     * bus.  For wide parts used in x8 mode, CFI query responses
     * are different than native byte-wide parts.
     */
    resp = pfl->cfi_table[boff];
    if (pfl->device_width != pfl->max_device_width) {
        /* The only case currently supported is x8 mode for a
         * wider part.
         */
        if (pfl->device_width != 1 || pfl->bank_width > 4) {
            DPRINTF("%s: Unsupported device configuration: "
                    "device_width=%d, max_device_width=%d\n",
                    __func__, pfl->device_width,
                    pfl->max_device_width);
            return 0;
        }
        /* CFI query data is repeated, rather than zero padded for
         * wide devices used in x8 mode.
         */
        for (i = 1; i < pfl->max_device_width; i++) {
            resp = deposit32(resp, 8 * i, 8, pfl->cfi_table[boff]);
        }
    }
    /* Replicate responses for each device in bank. */
    if (pfl->device_width < pfl->bank_width) {
        for (i = pfl->device_width;
             i < pfl->bank_width; i += pfl->device_width) {
            resp = deposit32(resp, 8 * i, 8 * pfl->device_width, resp);
        }
    }

    return resp;
}



/* Perform a device id query based on the bank width of the flash. */
static uint32_t pflash_devid_query(pflash_t *pfl, hwaddr offset)
{
    int i;
    uint32_t resp;
    hwaddr boff;

    /* Adjust incoming offset to match expected device-width
     * addressing. Device ID read addresses are always specified in
     * terms of the maximum supported width of the device.  This means
     * that x8 devices and x8/x16 devices in x8 mode behave
     * differently. For devices that are not used at their max width,
     * we will be provided with addresses that use higher address bits
     * than expected (based on the max width), so we will shift them
     * lower so that they will match the addresses used when
     * device_width==max_device_width.
     */
    boff = offset >> (ctz32(pfl->bank_width) +
                      ctz32(pfl->max_device_width) - ctz32(pfl->device_width));

    /* Mask off upper bits which may be used in to query block
     * or sector lock status at other addresses.
     * Offsets 2/3 are block lock status, is not emulated.
     */
    switch (boff & 0xFF) {
    case 0:
        resp = pfl->ident0;
        DPRINTF("%s: Manufacturer Code %04x\n", __func__, resp);
        break;
    case 1:
        resp = pfl->ident1;
        DPRINTF("%s: Device ID Code %04x\n", __func__, resp);
        break;
    default:
        DPRINTF("%s: Read Device Information offset=%x\n", __func__,
                (unsigned)offset);
        return 0;
        break;
    }
    /* Replicate responses for each device in bank. */
    if (pfl->device_width < pfl->bank_width) {
        for (i = pfl->device_width;
              i < pfl->bank_width; i += pfl->device_width) {
            resp = deposit32(resp, 8 * i, 8 * pfl->device_width, resp);
        }
    }

    return resp;
}

static uint32_t pflash_read (pflash_t *pfl, hwaddr offset,
                             int width, int be)
{
    hwaddr boff;
    uint32_t ret;
    uint8_t *p;

    ret = -1;

    uint8_t bank = offset / pfl->bank_size;

    //DPRINTF("%s: reading offset " TARGET_FMT_plx " under cmd %02x width %d\n",
    //        __func__, offset, pfl->cmd[bank], width);

    switch (pfl->cmd[bank]) {
    default:
        /* This should never happen : reset state & treat it as a read */
        fprintf(stderr, "%s: unknown command state: %x\n", __func__, pfl->cmd[bank]);
        pfl->wcycle[bank] = 0;
        pfl->cmd[bank] = 0;
        pfl->global_cmd = 0;
        /* fall through to read code */
    case 0x00:
        /* Flash area read */
        p = pfl->storage;
        switch (width) {
        case 1:
            ret = p[offset];
            //DPRINTF("%s: data offset " TARGET_FMT_plx " %02x\n",
            //        __func__, offset, ret);
            break;
        case 2:
            if (be) {
                ret = p[offset] << 8;
                ret |= p[offset + 1];
            } else {
                ret = p[offset];
                ret |= p[offset + 1] << 8;
            }
            //DPRINTF("%s: data offset " TARGET_FMT_plx " %04x\n",
            //        __func__, offset, ret);
            break;
        case 4:
            if (be) {
                ret = p[offset] << 24;
                ret |= p[offset + 1] << 16;
                ret |= p[offset + 2] << 8;
                ret |= p[offset + 3];
            } else {
                ret = p[offset];
                ret |= p[offset + 1] << 8;
                ret |= p[offset + 2] << 16;
                ret |= p[offset + 3] << 24;
            }
            //DPRINTF("%s: data offset " TARGET_FMT_plx " %08x\n",
            //        __func__, offset, ret);
            break;
        default:
            DPRINTF("BUG in %s\n", __func__);
        }

        break;
    case 0x70: /* Status Register */
        /* Status register read.  Return status from each device in
         * bank.
         */
        ret = pfl->status;
        if (pfl->device_width && width > pfl->device_width) {
            int shift = pfl->device_width * 8;
            while (shift + pfl->device_width * 8 <= width * 8) {
                ret |= pfl->status << shift;
                shift += pfl->device_width * 8;
            }
        } else if (!pfl->device_width && width > 2) {
            /* Handle 32 bit flash cases where device width is not
             * set. (Existing behavior before device width added.)
             */
            ret |= pfl->status << 16;
        }
        DPRINTF("%s: status %x\n", __func__, ret);
        pfl->cmd[bank] = 0;
        break;
    case 0x90:
        if (!pfl->device_width) {
            /* Preserve old behavior if device width not specified */
            boff = offset & 0xFF;
            if (pfl->bank_width == 2) {
                boff = boff >> 1;
            } else if (pfl->bank_width == 4) {
                boff = boff >> 2;
            }

            switch (boff) {
            case 0:
                ret = pfl->ident0 << 8 | pfl->ident1;
                DPRINTF("%s: Manufacturer Code %04x\n", __func__, ret);
                break;
            case 1:
                ret = pfl->ident2 << 8 | pfl->ident3;
                DPRINTF("%s: Device ID Code %04x\n", __func__, ret);
                break;
            default:
                DPRINTF("%s: Read Device Information boff=%x\n", __func__,
                        (unsigned)boff);
                ret = 0;
                break;
            }
        } else {
            /* If we have a read larger than the bank_width, combine multiple
             * manufacturer/device ID queries into a single response.
             */
            int i;
            for (i = 0; i < width; i += pfl->bank_width) {
                ret = deposit32(ret, i * 8, pfl->bank_width * 8,
                                pflash_devid_query(pfl,
                                                 offset + i * pfl->bank_width));
            }
        }
        break;
    case 0x98: /* Query mode */
        if (!pfl->device_width) {
            /* Preserve old behavior if device width not specified */
            boff = offset & 0xFF;
            if (pfl->bank_width == 2) {
                boff = boff >> 1;
            } else if (pfl->bank_width == 4) {
                boff = boff >> 2;
            }

            if (boff > pfl->cfi_len) {
                ret = 0;
            } else {
                ret = pfl->cfi_table[boff];
            }
        } else {
            /* If we have a read larger than the bank_width, combine multiple
             * CFI queries into a single response.
             */
            int i;
            for (i = 0; i < width; i += pfl->bank_width) {
                ret = deposit32(ret, i * 8, pfl->bank_width * 8,
                                pflash_cfi_query(pfl,
                                                 offset + i * pfl->bank_width));
            }
        }

        break;

    case 0xd0:  /* Configuration register */
        ret = pfl->configuration_register;     /* default configuration */
        break;
    }

    //DPRINTF("%s: returning 0x%x\n", __func__, ret);
    return ret;
}

/* update flash content on disk */
static void pflash_update(pflash_t *pfl, int offset,
                          int size)
{
    int offset_end;
    if (pfl->blk) {
        offset_end = offset + size;
        /* round to sectors */
        offset = QEMU_ALIGN_DOWN(offset, BDRV_SECTOR_SIZE);
        offset_end = QEMU_ALIGN_UP(offset_end, BDRV_SECTOR_SIZE);
        blk_pwrite(pfl->blk, offset, pfl->storage + offset ,
                   offset_end - offset,0);
    }
}

static inline void pflash_data_write(pflash_t *pfl, hwaddr offset,
                                     uint32_t value, int width, int be)
{
    uint8_t *p = pfl->storage;

    DPRINTF("%s: block write offset " TARGET_FMT_plx
            " value %x width %d counter %016" PRIx64 "\n",
            __func__, offset, value, width, pfl->counter);

    // NOTE: Flash can only flip 1's to 0's, so use a &= operation to update the
    // flash contents. This is critical because some drivers assume they can write
    // 0xFF to preserve the original value.
    switch (width) {
    case 1:
        p[offset] &= value;
        break;
    case 2:
        if (be) {
            p[offset] &= value >> 8;
            p[offset + 1] &= value;
        } else {
            p[offset] &= value;
            p[offset + 1] &= value >> 8;
        }
        break;
    case 4:
        if (be) {
            p[offset] &= value >> 24;
            p[offset + 1] &= value >> 16;
            p[offset + 2] &= value >> 8;
            p[offset + 3] &= value;
        } else {
            p[offset] &= value;
            p[offset + 1] &= value >> 8;
            p[offset + 2] &= value >> 16;
            p[offset + 3] &= value >> 24;
        }
        break;
    }

}

static void pflash_write(pflash_t *pfl, hwaddr offset,
                         uint32_t value, int width, int be)
{
    uint8_t cmd;
    int i;
    uint8_t *p;

    cmd = value;

    uint8_t bank = pfl->global_cmd ? 0 : offset / pfl->bank_size;
    uint32_t sector_offset = offset & (pfl->sector_len - 1);
    sector_offset = sector_offset >> (pfl->bank_width - 1);

    DPRINTF("%s: writing offset 0x%llx sector offset 0x%x value 0x%x width %d "
            "pfl->wcycle %d pfl->cmd 0x%x\n", __func__, offset, sector_offset, value, width,
            pfl->wcycle[bank], pfl->cmd[bank]);

    if (!pfl->wcycle[bank]) {
        /* Set the device in I/O access mode */
        memory_region_rom_device_set_romd(&pfl->mem, false);
    }

    switch (pfl->wcycle[bank]) {
    case 0:
        /* read mode */
        switch (cmd) {
        case 0x00: /* ??? */
            goto reset_bank;
        case 0x25:
            DPRINTF("%s: Program to buffer\n", __func__);
            pfl->status |= 0x80; /* Ready! */
            break;
        // Not implemented: 0x29 Buffer to Flash
        // Not implemented: 0x30 Erase Resume
        case 0x33:
            DPRINTF("%s: Blank check\n", __func__);
            // Is it blank?
            pfl->status |= 0x80; /* Ready! */
            pfl->status &= ~0x20;  /* clear non-erased bit */
            uint8_t *p = pfl->storage;
            offset &= ~(pfl->sector_len - 1);
            p += offset;
            for (i=0; i<pfl->sector_len; i++) {
                if (p[i] != 0xFF) {
                    pfl->status |= 0x20;    // Not erased
                    break;
                }
            }
            goto reset_bank;
        // Not implemented: 0x40 SSR Lock Entry
        // Not implemented: 0x50 Program Resume
        // Not implemented: 0x51 Program Suspend
        case 0x60: /* Sector Lock/Unlock */
            pfl->global_cmd = 0x60;
            bank = 0;
            DPRINTF("%s: Sector Lock/Unlock\n", __func__);
            break;
        case 0x70: /* Status Register */
            DPRINTF("%s: Read status register\n", __func__);
            pfl->cmd[bank] = cmd;
            return;
        case 0x71: /* Clear status bits */
            DPRINTF("%s: Clear status bits\n", __func__);
            pfl->status = 0x80;   // set device ready bit
            goto reset_bank;
        case 0x80: /* Erase setup */
            DPRINTF("%s: Erase setup\n", __func__);
            break;
        // Not implemented: 0x88 Secure Silicon Region Entry
        case 0x90: /* Read Device ID */
            DPRINTF("%s: Read Device information\n", __func__);
            pfl->cmd[bank] = cmd;
            return;
        case 0x98: /* CFI query */
            DPRINTF("%s: CFI query\n", __func__);
            break;
        // Not implemented: 0xB0 Erase Suspend
        case 0xd0: /* Enter configuration register */
            DPRINTF("%s: Configuration register enter\n", __func__);
            break;
        case 0xf0: /* Reset */
            DPRINTF("%s: Reset\n", __func__);
            pflash_reset_state(pfl);
            goto reset_bank;
        case 0xff: /* Read array mode */
            DPRINTF("%s: Read array mode\n", __func__);
            goto reset_bank;
        default:
            goto error_flash;
        }
        pfl->wcycle[bank]++;
        pfl->cmd[bank] = cmd;
        break;

    case 1:
        switch (pfl->cmd[bank]) {
        case 0x25:
            /* Mask writeblock size based on device width, or bank width if
             * device width not specified.
             */
            pfl->counter = value + 1;
            DPRINTF("%s: Program to buffer of %lld words\n", __func__, pfl->counter);
            pfl->wcycle[bank]++;
            break;
        case 0x60: /* Sector Lock... Expecting 0x288 = 60 */
            if (cmd == 0x60) {
                pfl->cmd[bank] = 0x60;
                pfl->wcycle[bank]++;
            } else {
                goto reset_bank;
            }
            break;
        case 0x80:
            if (sector_offset == 0x2aa && cmd == 0x30) {
                // sector erase
                offset &= ~(pfl->sector_len - 1);
                DPRINTF("%s: sector erase at " TARGET_FMT_plx " bytes %x\n",
                    __func__, offset, (unsigned)pfl->sector_len);

                if (!pfl->ro) {
                    p = pfl->storage;
                    memset(p + offset, 0xff, pfl->sector_len);
                    pflash_update(pfl, offset, pfl->sector_len);
                    pfl->status &= ~0x20;  /* clear non-erased bit */
                } else {
                    pfl->status |= 0x20; /* Block erase error */
                }
                pfl->status |= 0x80; /* Ready! */
            } else if (sector_offset == 0x2aa && cmd == 0x10) {
                DPRINTF("%s: chip erase", __func__);
                if (!pfl->ro) {
                    memset(pfl->storage, 0xff, pfl->sector_len * pfl->nb_blocs);
                    pflash_update(pfl, 0, pfl->sector_len * pfl->nb_blocs);
                    pfl->status &= ~0x20;  /* clear non-erased bit */
                } else {
                    pfl->status |= 0x20; /* Block erase error */
                }
                pfl->status |= 0x80; /* Ready! */
            } else {
                DPRINTF("%s: Unexpected command byte: 0x%x\n", __func__, cmd);
            }
            goto reset_bank;
            break;
        case 0x98:
            if (cmd == 0xf0) {
                DPRINTF("%s: leaving query mode\n", __func__);
                goto reset_bank;
            } else {
                DPRINTF("%s: Unexpected command byte: 0x%x\n", __func__, cmd);
                goto reset_bank;
            }
        case 0xd0:
            if (cmd == 0xf0) {
                DPRINTF("%s: leaving configuration register mode\n", __func__);
                pfl->status |= 0x80;
                goto reset_bank;
            } else if (cmd == 0x25) {
                /* program to buffer */
                DPRINTF("%s: 2nd byte of program to config register buffer\n", __func__);
                pfl->wcycle[bank]++;
            } else if (cmd == 0x29) {
                DPRINTF("%s: program config register buffer to flash \n", __func__);
                pfl->wcycle[bank] = 1;
            } else {
                DPRINTF("%s: Unexpected command byte: 0x%x\n", __func__, cmd);
                goto reset_bank;
            }
            break;
        default:
            goto error_flash;
        }
        break;

    case 2:
        switch (pfl->cmd[bank]) {
        case 0x25: /* Program to buffer  */
            if (!pfl->ro) {
                DPRINTF("%s: Programming %d bytes at " TARGET_FMT_plx " to 0x%x\n", __func__,
                            width, offset, value);
                pflash_data_write(pfl, offset, value, width, be);
            } else {
                pfl->status |= 0x10; /* Programming error */
            }
            pfl->status |= 0x80;

            pfl->counter--;
            if (!pfl->counter) {
                hwaddr mask = pfl->writeblock_size - 1;
                mask = ~mask;

                DPRINTF("%s: Programming finished\n", __func__);
                pfl->wcycle[bank]++;
                if (!pfl->ro) {
                    /* Flush the entire write buffer onto backing storage.  */
                    pflash_update(pfl, offset & mask, pfl->writeblock_size);
                } else {
                    pfl->status |= 0x10; /* Programming error */
                }
            }

            break;
        case 0xd0:
            if (sector_offset == 0x2aa && cmd == 0) {
                DPRINTF("%s: 3rd byte of program to config register buffer\n", __func__);
                pfl->wcycle[bank]++;
            } else {
                DPRINTF("%s: Unexpected command byte: 0x%x\n", __func__, cmd);
                goto reset_bank;
            }
            break;
        case 0x60: /* Sector Lock... expecting SLA = 0x60 (lock/unlock) or SLA = 0x61 (range) */
            // we don't implement locking but we should at least accept the command
            if (cmd == 0x60) {
                DPRINTF("%s: 3rd byte of Sector Lock\n", __func__);
                goto reset_bank;  // command is finished
            } else if (cmd == 0x61) {  // 0X61 multi sector lock
                pfl->wcycle[bank]++;
                pfl->cmd[bank] = 0x61;
            } else {
              goto error_flash;
            }
            break;
        default:
            goto error_flash;
        }
        break;

    case 3: /* Confirm mode */
        switch (pfl->cmd[bank]) {
        case 0x25: /* Block write */
            if (cmd == 0x29 && sector_offset == 0x555) {
                pfl->wcycle[bank] = 0;
                pfl->status |= 0x80;
            } else {
                PFLASH_BUG("%s: unknown command for Programming\n", __func__);
                goto reset_bank;
            }
            break;
        case 0x61: /* Sector Lock... expecting SLA = 0x61 */
            // we don't implement locking but we should at least accept the command
            if (cmd == 0x61) {
                DPRINTF("%s: 4th byte of Sector Lock\n", __func__);
                goto reset_bank;  // command is finished
            }
            goto error_flash;
            break;
        case 0xd0:
            if (sector_offset == 0) {
                pfl->configuration_register = value;
                DPRINTF("%s: setting new config regsistr: 0x%x\n", __func__, value);
                pfl->wcycle[bank] = 1;
            } else {
                DPRINTF("%s: Unexpected command byte: 0x%x\n", __func__, cmd);
                goto reset_bank;
            }
            break;
        default:
            goto error_flash;
        }
        break;
    default:
        /* Should never happen */
        DPRINTF("%s: invalid write state\n",  __func__);
        goto reset_bank;
    }
    return;

 error_flash:
    fprintf(stderr, "PFLASH %s: Unimplemented flash cmd sequence "
                  "(offset 0x%lx sector offset 0x%x, bank %u pfl->wcycle %d pfl->cmd 0x%x value 0x%x)"
                  "\n", __func__, offset, sector_offset, bank, pfl->wcycle[bank], pfl->cmd[bank], value);

 reset_bank:
    memory_region_rom_device_set_romd(&pfl->mem, true);

    pfl->wcycle[bank] = 0;
    pfl->cmd[bank] = 0;
    pfl->global_cmd = 0;
}


static uint32_t pflash_readb_be(void *opaque, hwaddr addr)
{
    return pflash_read(opaque, addr, 1, 1);
}

static uint32_t pflash_readb_le(void *opaque, hwaddr addr)
{
    return pflash_read(opaque, addr, 1, 0);
}

static uint32_t pflash_readw_be(void *opaque, hwaddr addr)
{
    pflash_t *pfl = opaque;

    return pflash_read(pfl, addr, 2, 1);
}

static uint32_t pflash_readw_le(void *opaque, hwaddr addr)
{
    pflash_t *pfl = opaque;

    return pflash_read(pfl, addr, 2, 0);
}

static uint32_t pflash_readl_be(void *opaque, hwaddr addr)
{
    pflash_t *pfl = opaque;

    return pflash_read(pfl, addr, 4, 1);
}

static uint32_t pflash_readl_le(void *opaque, hwaddr addr)
{
    pflash_t *pfl = opaque;

    return pflash_read(pfl, addr, 4, 0);
}

static void pflash_writeb_be(void *opaque, hwaddr addr,
                             uint32_t value)
{
    pflash_write(opaque, addr, value, 1, 1);
}

static void pflash_writeb_le(void *opaque, hwaddr addr,
                             uint32_t value)
{
    pflash_write(opaque, addr, value, 1, 0);
}

static void pflash_writew_be(void *opaque, hwaddr addr,
                             uint32_t value)
{
    pflash_t *pfl = opaque;

    pflash_write(pfl, addr, value, 2, 1);
}

static void pflash_writew_le(void *opaque, hwaddr addr,
                             uint32_t value)
{
    pflash_t *pfl = opaque;

    pflash_write(pfl, addr, value, 2, 0);
}

static void pflash_writel_be(void *opaque, hwaddr addr,
                             uint32_t value)
{
    pflash_t *pfl = opaque;

    pflash_write(pfl, addr, value, 4, 1);
}

static void pflash_writel_le(void *opaque, hwaddr addr,
                             uint32_t value)
{
    pflash_t *pfl = opaque;

    pflash_write(pfl, addr, value, 4, 0);
}

static const MemoryRegionOps pflash_jedec_ops_be = {
    .old_mmio = {
        .read = { pflash_readb_be, pflash_readw_be, pflash_readl_be, },
        .write = { pflash_writeb_be, pflash_writew_be, pflash_writel_be, },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps pflash_jedec_ops_le = {
    .old_mmio = {
        .read = { pflash_readb_le, pflash_readw_le, pflash_readl_le, },
        .write = { pflash_writeb_le, pflash_writew_le, pflash_writel_le, },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void pflash_jedec_realize(DeviceState *dev, Error **errp)
{
    pflash_t *pfl = CFI_PFLASH_JEDEC(dev);
    uint64_t total_len;
    int ret;
    uint64_t blocks_per_device, device_len;
    int num_devices;

    total_len = pfl->sector_len * pfl->nb_blocs;

    /* These are only used to expose the parameters of each device
     * in the cfi_table[].
     */
    num_devices = pfl->device_width ? (pfl->bank_width / pfl->device_width) : 1;
    blocks_per_device = pfl->nb_blocs / num_devices;
    device_len = pfl->sector_len * blocks_per_device;

    memory_region_init_rom_device(
        &pfl->mem, OBJECT(dev),
        pfl->be ? &pflash_jedec_ops_be : &pflash_jedec_ops_le, pfl,
        pfl->name, total_len, errp);
    vmstate_register_ram(&pfl->mem, DEVICE(pfl));
    pfl->storage = memory_region_get_ram_ptr(&pfl->mem);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &pfl->mem);

    if (pfl->blk) {
        /* read the initial flash content */
        ret = blk_pread(pfl->blk, 0, pfl->storage, total_len);

        if (ret < 0) {
            vmstate_unregister_ram(&pfl->mem, DEVICE(pfl));
            error_setg(errp, "failed to read the initial flash content");
            return;
        }
    }

    if (pfl->blk) {
        pfl->ro = blk_is_read_only(pfl->blk);
    } else {
        pfl->ro = 0;
    }

    /* Default to devices being used at their maximum device width. This was
     * assumed before the device_width support was added.
     */
    if (!pfl->max_device_width) {
        pfl->max_device_width = pfl->device_width;
    }

    pfl->configuration_register = 0xdf48;

    pflash_reset_state(pfl);
    pfl->status = 0;
    /* Hardcoded CFI table */
    pfl->cfi_len = 0x52;
    /* Standard "QRY" string */
    pfl->cfi_table[0x10] = 'Q';
    pfl->cfi_table[0x11] = 'R';
    pfl->cfi_table[0x12] = 'Y';
    /* Command set (JEDEC 42.4) */
    pfl->cfi_table[0x13] = 0x02;
    pfl->cfi_table[0x14] = 0x00;
    /* Primary extended table address (none) */
    pfl->cfi_table[0x15] = 0x40;
    pfl->cfi_table[0x16] = 0x00;
    /* Alternate command set (none) */
    pfl->cfi_table[0x17] = 0x00;
    pfl->cfi_table[0x18] = 0x00;
    /* Alternate extended table (none) */
    pfl->cfi_table[0x19] = 0x00;
    pfl->cfi_table[0x1A] = 0x00;
    /* Vcc min */
    pfl->cfi_table[0x1B] = 0x17;
    /* Vcc max */
    pfl->cfi_table[0x1C] = 0x19;
    /* Vpp min (no Vpp pin) */
    pfl->cfi_table[0x1D] = 0x00;
    /* Vpp max (no Vpp pin) */
    pfl->cfi_table[0x1E] = 0x00;
    /* Reserved */
    pfl->cfi_table[0x1F] = 0x04;
    /* Timeout for min size buffer write */
    pfl->cfi_table[0x20] = 0x09;
    /* Typical timeout for block erase */
    pfl->cfi_table[0x21] = 0x0a;
    /* Typical timeout for full chip erase (4096 ms) */
    pfl->cfi_table[0x22] = 0x11;
    /* Reserved */
    pfl->cfi_table[0x23] = 0x04;
    /* Max timeout for buffer write */
    pfl->cfi_table[0x24] = 0x02;
    /* Max timeout for block erase */
    pfl->cfi_table[0x25] = 0x03;
    /* Max timeout for chip erase */
    pfl->cfi_table[0x26] = 0x00;
    /* Device size */
    pfl->cfi_table[0x27] = ctz32(device_len); /* + 1; */
    /* Flash device interface (8 & 16 bits) */
    pfl->cfi_table[0x28] = 0x01;
    pfl->cfi_table[0x29] = 0x00;
    /* Max number of bytes in multi-bytes write */
    if (pfl->bank_width == 1) {
        pfl->cfi_table[0x2A] = 0x06;
    } else {
        pfl->cfi_table[0x2A] = 0x06;
    }
    pfl->writeblock_size = 1 << pfl->cfi_table[0x2A];

    pfl->cfi_table[0x2B] = 0x00;
    /* Number of erase block regions (uniform) */
    pfl->cfi_table[0x2C] = 0x02;
    /* Erase block region 1 */
    pfl->cfi_table[0x2D] = 3;
    pfl->cfi_table[0x2E] = 0;
    pfl->cfi_table[0x2F] = pfl->sector_len >> 8;
    pfl->cfi_table[0x30] = pfl->sector_len >> 16;
    /* Erase block region 2 */
    pfl->cfi_table[0x31] = 0x7e;
    pfl->cfi_table[0x32] = 0;
    pfl->cfi_table[0x33] = 0;
    pfl->cfi_table[0x34] = 0x02;


    /* Extended */
    pfl->cfi_table[0x40] = 'P';
    pfl->cfi_table[0x41] = 'R';
    pfl->cfi_table[0x42] = 'I';

    pfl->cfi_table[0x43] = '1';
    pfl->cfi_table[0x44] = '3';

    pfl->cfi_table[0x45] = 0x00;
    pfl->cfi_table[0x46] = 0x02;
    pfl->cfi_table[0x47] = 0x01;
    pfl->cfi_table[0x48] = 0x00;

    pfl->cfi_table[0x49] = 0x08;
    pfl->cfi_table[0x4a] = 0x00;
    pfl->cfi_table[0x4b] = 0x01;
    pfl->cfi_table[0x4c] = 0x00;
    pfl->cfi_table[0x4d] = 0x85;
    pfl->cfi_table[0x4e] = 0x95;
    pfl->cfi_table[0x4f] = 0x02;
    pfl->cfi_table[0x50] = 0x01;

}

static Property pflash_jedec_properties[] = {
    DEFINE_PROP_DRIVE("drive", struct pflash_t, blk),
    /* num-blocks is the number of blocks actually visible to the guest,
     * ie the total size of the device divided by the sector length.
     * If we're emulating flash devices wired in parallel the actual
     * number of blocks per indvidual device will differ.
     */
    DEFINE_PROP_UINT32("num-blocks", struct pflash_t, nb_blocs, 0),
    DEFINE_PROP_UINT64("sector-length", struct pflash_t, sector_len, 0),
    /* width here is the overall width of this QEMU device in bytes.
     * The QEMU device may be emulating a number of flash devices
     * wired up in parallel; the width of each individual flash
     * device should be specified via device-width. If the individual
     * devices have a maximum width which is greater than the width
     * they are being used for, this maximum width should be set via
     * max-device-width (which otherwise defaults to device-width).
     * So for instance a 32-bit wide QEMU flash device made from four
     * 16-bit flash devices used in 8-bit wide mode would be configured
     * with width = 4, device-width = 1, max-device-width = 2.
     *
     * If device-width is not specified we default to backwards
     * compatible behaviour which is a bad emulation of two
     * 16 bit devices making up a 32 bit wide QEMU device. This
     * is deprecated for new uses of this device.
     */
    DEFINE_PROP_UINT8("width", struct pflash_t, bank_width, 0),
    DEFINE_PROP_UINT8("device-width", struct pflash_t, device_width, 0),
    DEFINE_PROP_UINT8("max-device-width", struct pflash_t, max_device_width, 0),
    DEFINE_PROP_UINT32("bank-size", struct pflash_t, bank_size, 0),
    DEFINE_PROP_UINT8("big-endian", struct pflash_t, be, 0),
    DEFINE_PROP_UINT16("id0", struct pflash_t, ident0, 0),
    DEFINE_PROP_UINT16("id1", struct pflash_t, ident1, 0),
    DEFINE_PROP_UINT16("id2", struct pflash_t, ident2, 0),
    DEFINE_PROP_UINT16("id3", struct pflash_t, ident3, 0),
    DEFINE_PROP_STRING("name", struct pflash_t, name),
    DEFINE_PROP_END_OF_LIST(),
};

static void pflash_jedec_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = pflash_jedec_realize;
    dc->props = pflash_jedec_properties;
    dc->vmsd = &vmstate_pflash;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}


static const TypeInfo pflash_jedec_info = {
    .name           = TYPE_CFI_PFLASH_JEDEC_424,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(struct pflash_t),
    .class_init     = pflash_jedec_class_init,
};

static void pflash_jedec_register_types(void)
{
    type_register_static(&pflash_jedec_info);
}

type_init(pflash_jedec_register_types)

pflash_t *pflash_jedec_424_register(hwaddr base,
                                DeviceState *qdev, const char *name,
                                hwaddr size,
                                BlockBackend *blk,
                                uint32_t sector_len, int nb_blocs, uint32_t bank_size,
                                int bank_width, uint16_t id0, uint16_t id1,
                                uint16_t id2, uint16_t id3, int be)
{
    DeviceState *dev = qdev_create(NULL, TYPE_CFI_PFLASH_JEDEC_424);

    if (blk) {
    	qdev_prop_set_drive(dev, "drive", blk,&error_abort);
    }
    qdev_prop_set_uint32(dev, "num-blocks", nb_blocs);
    qdev_prop_set_uint64(dev, "sector-length", sector_len);
    qdev_prop_set_uint8(dev, "width", bank_width);
    qdev_prop_set_uint32(dev, "bank-size", bank_size);
    qdev_prop_set_uint8(dev, "big-endian", !!be);
    qdev_prop_set_uint16(dev, "id0", id0);
    qdev_prop_set_uint16(dev, "id1", id1);
    qdev_prop_set_uint16(dev, "id2", id2);
    qdev_prop_set_uint16(dev, "id3", id3);
    qdev_prop_set_string(dev, "name", name);
    qdev_init_nofail(dev);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
    return CFI_PFLASH_JEDEC(dev);
}

MemoryRegion *pflash_jedec_424_get_memory(pflash_t *fl)
{
    return &fl->mem;
}
