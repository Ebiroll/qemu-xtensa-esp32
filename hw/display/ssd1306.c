/*
 * SSD1306 OLED controller with 128x64 display.
 *
 * Copyright (c) 2017 Olof Astrand
 * Copyright (c) 2006-2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

/* The controller can support a variety of different displays, but we only
   implement one.  Most of the commends relating to brightness and geometry
   setup are ignored. */
#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "ui/console.h"

#define DEBUG_SSD1306 1

#ifdef DEBUG_SSD1306
#define DPRINTF(fmt, ...) \
do { printf("ssd1306: " fmt , ## __VA_ARGS__); } while (0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "ssd1306: error: " fmt , ## __VA_ARGS__); exit(1);} while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
do { fprintf(stderr, "ssd1306: error: " fmt , ## __VA_ARGS__);} while (0)
#endif

/* Scaling factor for pixels.  */
#define MAGNIFY 4

enum ssd1306_mode
{
    SSD1306_IDLE,
    SSD1306_DATA,
    SSD1306_CMD
};

enum ssd1306_adressing_mode
{
    SSD1306_HORIZONTAL=0,
    SSD1306_VERTICAL,
    SSD1306_PAGE,
    SSD1306_INVALID
};


enum ssd1306_cmd {
    SSD1306_CMD_NONE,
    SSD1306_CMD_SKIP1,
    SSD1306_CMD_SKIP2,
    SSD1306_COLUMN_ADRESSING1,
    SSD1306_COLUMN_ADRESSING2,
    SSD1306_ROW_ADRESSING1,
    SSD1306_ROW_ADRESSING2
};

#define TYPE_SSD1306 "ssd1306"
#define SSD1306(obj) OBJECT_CHECK(ssd1306_state, (obj), TYPE_SSD1306)

#define MAX_FRAMEBUFF (132*64)

typedef struct {
    I2CSlave parent_obj;

    QemuConsole *con;
    int row;
    int col;

    int col_min;
    int col_max;
    int row_min;
    int row_max;

    int start_line;
    int mirror;
    int flash;
    int enabled;
    int inverse;
    int redraw;
    enum ssd1306_mode mode;
    enum ssd1306_adressing_mode  adressing_mode;
    enum ssd1306_cmd cmd_state;
    uint8_t framebuffer[MAX_FRAMEBUFF];
} ssd1306_state;


ssd1306_state *the_ssd1306=NULL;


static int ssd1306_recv(I2CSlave *i2c)
{
    BADF("Reads not implemented\n");
    return -1;
}

static int ssd1306_send(I2CSlave *i2c, uint8_t data)
{
    ssd1306_state *s = the_ssd1306; //SSD1306(i2c);
    enum ssd1306_cmd old_cmd_state;

    switch (s->mode) {
    case SSD1306_IDLE:
        DPRINTF("ssd1306 byte 0x%02x\n", data);
        //if (data == 0x00)
        //    s->mode = SSD1306_CMD;
        if (data == 0x80)
            s->mode = SSD1306_CMD;
        else if (data == 0x40)
            s->mode = SSD1306_DATA;
        else
            //BADF("Unexpected byte 0x%x\n", data);
            DPRINTF(" ssd1306 Unexpected byte 0x%x\n", data);
        break;
    case SSD1306_DATA:
        {
            DPRINTF("data 0x%02x ", data);
            int offset=0;
            //s->framebuffer[offset] = 0xf0;
            //s->framebuffer[offset+1] =  0xf0;
            //s->framebuffer[offset+2] =  0xf0;

            //if (s->adressing_mode==SSD1306_PAGE) {
            //    offset=s->col + s->row * 128;
            //    if (offset < MAX_FRAMEBUFF) {
            //        s->framebuffer[offset] = data;
            //    }
            //    s->row++;
            //    if (s->row>7) {
            //        s->col++;
            //        s->row=0;
            //    }
            //} else if (s->adressing_mode==SSD1306_HORIZONTAL) {
                //if (s->row>7) {
                //    s->row=0;
               //}
                offset=s->col + s->row * 128;
                //DPRINTF("%d,%d offset 0x%02x\n", s->col , s->row,offset);
                if (offset < MAX_FRAMEBUFF) {
                    s->framebuffer[offset] = data;
                }
                s->col++;

                if (s->col>s->col_max) {
                    s->row++;
                    s->col=s->col_min;                    
                }
                if (s->row>s->row_max) {
                    //s->row=s->row_min;
                    // Seems to work like this (on cheap displays?)...
                    s->row=0;
                }

                //if (s->col>127) {
                //    s->row++;
                //    s->col=0;
                //}
            //}
            s->redraw = 1;
        }
        break;
    case SSD1306_CMD:
        old_cmd_state = s->cmd_state;
        s->cmd_state = SSD1306_CMD_NONE;
        switch (old_cmd_state) {
        case SSD1306_CMD_NONE:
        {
            DPRINTF("1306 cmd 0x%02x\n", data);
            s->mode = SSD1306_IDLE;
            switch (data) {
            case 0x00 ... 0x0f: /* Set lower column address.  */
                //if ( s->adressing_mode==SSD1306_PAGE) {
                    s->col = (s->col & 0xf0) | (data & 0xf);
                    DPRINTF("1306 col 0x%02x\n", s->col);

                //}
                break;
            case 0x10 ... 0x1f: /* Set higher column address.  */
                //if ( s->adressing_mode==SSD1306_PAGE) {
                    s->col = (s->col & 0x0f) | ((data & 0xf) << 4);
                    DPRINTF("1306 col 0x%02x\n", s->col);

                //}
                break;
            case 0x20:
                // Adressing mode,
                { 
                   DPRINTF("1306 Adressing mode 0x%02x\n", data & 0x03);

                    switch (data & 0x03) 
                    {
                        case 0:
                            DPRINTF("1306 page adressing mode Horizontal\n");
                            break;
                        case 1:
                            DPRINTF("1306 page adressing mode Vertical\n");
                            break;
                        case 2:
                            DPRINTF("1306 page adressing mode\n");
                            break;
          
                    }
                    // Arduino driver insert spurious data
                    s->adressing_mode=data & 0x03;
                }
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0x21:
                // Column Address , 1-127
                { 
                   DPRINTF("1306 Column addressing 0x%02x\n", data );
                   s->cmd_state = SSD1306_COLUMN_ADRESSING2;
                    //s->col = 0;
                }
                break;
            case 0x22:
                // Page Address , 0-7
                { 
                   DPRINTF("1306 Page addressing 0x%02x\n", data );
                   s->cmd_state = SSD1306_ROW_ADRESSING2;
                   //s->col = 0;

                }
                break;

            case 0x40 ... 0x7f: /* Set start line.  */
                 DPRINTF("1306 segment map 0x%02x\n", data );
                s->start_line = 0;
                break;
            case 0x81: /* Set contrast (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xa0: /* was Mirror off.  */
                DPRINTF("1306 segment map 0x%02x\n", data );
                //s->mirror = 0;
                break;
            case 0xa1: /* was Mirror on.  */
                DPRINTF("1306 segment remap 0x%02x\n", data );
                //s->mirror = 1;
                break;
            case 0xa4: /* Entire display off.  */
                s->flash = 0;
                break;
            case 0xa5: /* Entire display on.  */
                s->flash = 1;
                break;
            case 0xa6: /* Inverse off.  */
                s->inverse = 0;
                break;
            case 0xa7: /* Inverse on.  */
                s->inverse = 1;
                break;
            case 0xa8: /* Set multiplied ratio (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xad: /* DC-DC power control.  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xae: /* Display off.  */
                s->enabled = 0;
                break;
            case 0xaf: /* Display on.  */
                s->enabled = 1;
                break;
            case 0xb0 ... 0xbf: /* Set Page address.  */
                DPRINTF("1306 Set Page address 0x%02x\n", data );
                s->row = data & 7;
                //s->col=0;
                break;
            case 0xc0 ... 0xc8: /* Set COM output direction (Ignored).  */
                break;
            case 0xd3: /* Set display offset (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xd5: /* Set display clock (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xd8: /* Set color and power mode (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xd9: /* Set pre-charge period (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xda: /* Set COM pin configuration (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xdb: /* Set VCOM dselect level (Ignored).  */
                s->cmd_state = SSD1306_CMD_SKIP1;
                break;
            case 0xe3: /* no-op.  */
                break;
            default:
                break;
                //BADF("Unknown command: 0x%x\n", data);
            }
            }

            break;
        case SSD1306_ROW_ADRESSING2:
             DPRINTF("1306 ROW_ADRESSING2 0x%02x\n", data);
            s->cmd_state = SSD1306_ROW_ADRESSING1;
            s->row_min=data;
            s->row=data;      
            break;
        case SSD1306_ROW_ADRESSING1:
            DPRINTF("1306 ROW_ADRESSING1 0x%02x\n", data);
            s->row_max=data;  
            //s->col_min=s->col;
            //s->col_max=s->col;    
            break;

        case SSD1306_COLUMN_ADRESSING2:
             DPRINTF("1306 COLUMN_ADRESSING2 0x%02x\n", data);
            s->cmd_state = SSD1306_COLUMN_ADRESSING1;
            s->col_min=data;
            s->col=data;      
            break;
        case SSD1306_COLUMN_ADRESSING1:
            DPRINTF("1306 COLUMN_ADRESSING1 0x%02x\n", data);
            s->col_max=data;      
            break;

        case  SSD1306_CMD_SKIP2:
            DPRINTF("1306 skip2 0x%02x\n", data);
            s->cmd_state = SSD1306_CMD_SKIP1;
            break;

        case SSD1306_CMD_SKIP1:
            DPRINTF("1306 skip 0x%02x\n", data);
            break;
        }
        break;
    }
    return 0;
}

static void ssd1306_event(I2CSlave *i2c, enum i2c_event event)
{
    //ssd1306_state *s = SSD1306(i2c);
    ssd1306_state *s = the_ssd1306;

    DPRINTF("ssd1306_event 0x%02x\n", (int)event);

    switch (event) {
    case I2C_FINISH:
        s->mode = SSD1306_IDLE;
        break;
    case I2C_START_RECV:
        DPRINTF("ssd1306_I2C_START_RECV  0x%02x\n", (int)event);
    case I2C_START_SEND:
    case I2C_NACK:
        /* Nothing to do.  */
        break;
    }
}

static void ssd1306_update_display(void *opaque)
{
    //ssd1306_state *s = (ssd1306_state *)opaque;
    ssd1306_state *s = the_ssd1306;

    DisplaySurface *surface = qemu_console_surface(s->con);
    uint8_t *dest;
    uint8_t *src;
    int x;
    int y;
    int line;
    char *colors[2];
    char colortab[MAGNIFY * 8];
    int dest_width;
    uint8_t mask;

    //DPRINTF("ssd1306_update_display  0x%02x\n", 0x47);

    if (!s->redraw)
        return;

    switch (surface_bits_per_pixel(surface)) {
    case 0:
        return;
    case 15:
        dest_width = 2;
        break;
    case 16:
        dest_width = 2;
        break;
    case 24:
        dest_width = 3;
        break;
    case 32:
        dest_width = 4;
        break;
    default:
        BADF("Bad color depth\n");
        return;
    }
    dest_width *= MAGNIFY;
    memset(colortab, 0xff, dest_width);
    memset(colortab + dest_width, 0, dest_width);
    if (s->flash) {
        colors[0] = colortab;
        colors[1] = colortab;
    } else if (s->inverse) {
        colors[0] = colortab;
        colors[1] = colortab + dest_width;
    } else {
        colors[0] = colortab + dest_width;
        colors[1] = colortab;
    }
    dest = surface_data(surface);
    for (y = 0; y < 64; y++) {
        line = (y + s->start_line) & 63;
        src = s->framebuffer + 128 * (line >> 3) ;
        mask = 1 << (line & 7);
        for (x = 0; x < 128; x++) {
            memcpy(dest, colors[(*src & mask) != 0], dest_width);
            dest += dest_width;
            src++;
        }
        for (x = 1; x < MAGNIFY; x++) {
            memcpy(dest, dest - dest_width * 128, dest_width * 128);
            dest += dest_width * 128;
        }
    }
    s->redraw = 0;
    dpy_gfx_update(s->con, 0, 0, 128 * MAGNIFY, 64 * MAGNIFY);
}

static void ssd1306_invalidate_display(void * opaque)
{
    ssd1306_state *s = the_ssd1306; //(ssd1306_state *)opaque;
    s->redraw = 1;
}

static const VMStateDescription vmstate_ssd1306 = {
    .name = "ssd1306_oled",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_INT32(row, ssd1306_state),
        VMSTATE_INT32(col, ssd1306_state),
        VMSTATE_INT32(col, ssd1306_state),
        VMSTATE_INT32(col_min, ssd1306_state),
        VMSTATE_INT32(col_max, ssd1306_state),
        VMSTATE_INT32(row_min, ssd1306_state),
        VMSTATE_INT32(row_max, ssd1306_state),
        VMSTATE_INT32(start_line, ssd1306_state),
        VMSTATE_INT32(mirror, ssd1306_state),
        VMSTATE_INT32(flash, ssd1306_state),
        VMSTATE_INT32(enabled, ssd1306_state),
        VMSTATE_INT32(inverse, ssd1306_state),
        VMSTATE_INT32(redraw, ssd1306_state),
        VMSTATE_UINT32(mode, ssd1306_state),
        VMSTATE_UINT32(cmd_state, ssd1306_state),
        VMSTATE_BUFFER(framebuffer, ssd1306_state),
        VMSTATE_I2C_SLAVE(parent_obj, ssd1306_state),
        VMSTATE_END_OF_LIST()
    }
};

static const GraphicHwOps ssd1306_ops = {
    .invalidate  = ssd1306_invalidate_display,
    .gfx_update  = ssd1306_update_display,
};

static int ssd1306_init(I2CSlave *i2c)
{
    ssd1306_state *s = SSD1306(i2c);

    // Only one display although it can appear in many io-locations.
    if (the_ssd1306==NULL) {
        the_ssd1306=s;
    }

    s->col=0;
    s->row=0;
    s->col_max=127;
    s->col_min=0;

    s->row_max=7;
    s->row_min=0;

    s->start_line=0;
    s->adressing_mode=SSD1306_PAGE;

    s->con = graphic_console_init(DEVICE(i2c), 0, &ssd1306_ops, s);
    qemu_console_resize(s->con, 128 * MAGNIFY, 64 * MAGNIFY);
    int offset=0;
    while(offset<MAX_FRAMEBUFF) {
        s->framebuffer[offset] = 0 ;
        offset++;
    }



    return 0;
}

static void ssd1306_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    k->init = ssd1306_init;
    k->event = ssd1306_event;
    k->recv = ssd1306_recv;
    k->send = ssd1306_send;
    dc->vmsd = &vmstate_ssd1306;
}

static const TypeInfo ssd1306_info = {
    .name          = TYPE_SSD1306,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(ssd1306_state),
    .class_init    = ssd1306_class_init,
};

static void ssd1306_register_types(void)
{
    type_register_static(&ssd1306_info);
}

type_init(ssd1306_register_types)
