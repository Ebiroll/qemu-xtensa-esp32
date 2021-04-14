/*
 * nrf52_dk board
 *
 * Copyright 2018 Joel Stanley <joel@jms.id.au>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *   https://infocenter.nordicsemi.com/index.jsp?topic=%2Fmigration_nrf52%2FMIG%2Fnrf52_migration%2Fpreface.html
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/arm/arm.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"

#include "hw/arm/nrf52_soc.h"
#include "hw/arm/nrf52.h"
#include "hw/i2c/microbit_i2c.h"
#include "elf.h"
#include "hw/loader.h"


typedef struct {
    MachineState parent;

    NRF52State nrf52;
    MicrobitI2CState i2c;
} Nrf52MachineState;

#define TYPE_NRF52_MACHINE MACHINE_TYPE_NAME("nrf52_dk")

#define NRF52_MACHINE(obj) \
    OBJECT_CHECK(Nrf52MachineState, obj, TYPE_NRF52_MACHINE)


#define FLASH_ADDR_START (0x08000000)

// We try this mapping to get us started
static uint64_t translate_address(void *opaque, uint64_t from_addr)
{
    printf("MAP %" PRIx64 " \n",from_addr);

    if (from_addr == FLASH_ADDR_START) {
        return 0x00000000;
    }
    return from_addr;
}
/*
3d00 0008
*/
static void init_mbr(void);

static void init_mbr(void) {
// Lowest bit indicates thumb instruction..
   unsigned int  dummy_fn=0x0010001;
   unsigned int service_fn=0x020001;
   unsigned int i;

   for (i=0;i<20;i++) {
      cpu_physical_memory_write(i*4, &dummy_fn, 4 );
   }


/* dummy_fn
   0:	b580      	push	{r7, lr}
   2:	af00      	add	r7, sp, #0
   4:	46c0      	nop			; (mov r8, r8)
   6:	46bd      	mov	sp, r7
   8:	bc80      	pop	{r7}
   a:	bc01      	pop	{r0}
   c:	4700      	bx	r0
*/


    dummy_fn=0x0010000;
    i=dummy_fn;
    unsigned int prog=0xb580;
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0xaf00;
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;

    //  22:	2000      	movs	r0, #0
    //prog=0x2000;

    //  4:	46c0      	nop			; (mov r8, r8)
    prog=0x46c0;
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0x46bd;
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0xbc80;
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0xbc01;
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0x4700;
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;


    // Put 
    //service_fn=4*((i+8)/4);
    service_fn=0x0010040;
    i=service_fn;

    printf("servie function %u\n",service_fn); 


    service_fn=service_fn+1;  // Thumbs up 

    prog=0xb580;   // push	{r7, lr}
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0xaf00;  // add	r7, sp, #0
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;

    prog=0x2000;      	// movs	r0, #0
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;

    // This stores r0 to value
    prog=0x60b8;      	// str	r0, [r7, #8]
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;

    prog=0x46c0;    // nop 
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0x46bd;   // mov	sp, r7
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0xbc80;  // pop	{r7}
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0xbc01;  // pop	{r0}
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;
    prog=0x4700;  // bx	r0
    cpu_physical_memory_write(i, &prog, 2 );
    i+=2;


    // This is the one
    cpu_physical_memory_write(11*4, &service_fn, 4 );


    //prog=0xc046bd46;


}

static void nrf52_init(MachineState *machine)
{
    printf("INIT!!!\n\n");

    Nrf52MachineState *s = NRF52_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *mr;
    Object *soc = OBJECT(&s->nrf52);
    Object *i2c = OBJECT(&s->i2c);

    sysbus_init_child_obj(OBJECT(machine), "nrf52", soc, sizeof(s->nrf52),
                          TYPE_NRF52_SOC);
    qdev_prop_set_chr(DEVICE(&s->nrf52), "serial0", serial_hd(0));
    object_property_set_link(soc, OBJECT(system_memory), "memory",
                             &error_fatal);
    object_property_set_bool(soc, true, "realized", &error_fatal);

    /*
     * Overlap the TWI stub device into the SoC.  This is a microbit-specific
     * hack until we implement the nRF52 TWI controller properly and the
     * magnetometer/accelerometer devices.
     */
    sysbus_init_child_obj(OBJECT(machine), "nrf52.twi", i2c,
                          sizeof(s->i2c), TYPE_MICROBIT_I2C);
    object_property_set_bool(i2c, true, "realized", &error_fatal);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(i2c), 0);
    memory_region_add_subregion_overlap(&s->nrf52.container, NRF52_TWI_BASE,
                                        mr, -1);
    {
        MemoryRegion *hack = g_new(MemoryRegion, 1);


        // Hack to map an additional page of ram at the top of the address
        //space.  This stops qemu complaining about executing code outside RAM
        //when returning from an exception. 
        memory_region_init_ram(hack, NULL, "armv7m.hack", 0x1000, &error_fatal);
        //vmstate_register_ram_global(hack);
        //memory_region_add_subregion(system_memory, 0xfffff000, hack);
       memory_region_add_subregion_overlap(&s->nrf52.container, 0xfffff000, hack,-1);

    }




    uint64_t elf_entry;
    uint64_t elf_lowaddr;
    // ARM_CPU(first_cpu)
    int success = load_elf(machine->kernel_filename, NULL, translate_address, ARM_CPU(s->nrf52.cpu.cpu),
            &elf_entry, &elf_lowaddr, NULL, 0 , EM_ARM, 0, 0);

 // https://electronut.in/nrf52-baremetal/
    if (success>0) {
        printf("start addr %" PRIx64 "\n",elf_entry); 
        printf("low addr %" PRIx64 "\n",elf_lowaddr); 


        init_mbr();
        //s->nrf52.cpu->env.pc=elf_entry;
        //elf_lowaddr= NRF52_SRAM_BASE+64*1024;
        elf_lowaddr=0x20002000;
        cpu_physical_memory_write(0, &elf_lowaddr, 4 );
        cpu_physical_memory_write(4, &elf_entry, 4 );
    }
    {
        ARMCPU *cpu = ARM_CPU(s->nrf52.cpu.cpu);

        cpu_reset(CPU(cpu));
    }

    //armv7m_load_kernel(ARM_CPU(&s->nrf52.cpu)/*ARM_CPU(first_cpu)*/, machine->kernel_filename,
    //                   NRF52_SOC(soc)->flash_size);
}

static void nrf52_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "nrf52 dev kit";
    mc->init = nrf52_init;
    mc->max_cpus = 1;
}

static const TypeInfo microbit_info = {
    .name = TYPE_NRF52_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(Nrf52MachineState),
    .class_init = nrf52_machine_class_init,
};

static void microbit_machine_init(void)
{
    type_register_static(&microbit_info);
}

#if 0
void stm32f4xx_init(
            ram_addr_t flash_size,        /* in KBytes */
            ram_addr_t ram_size,          /* in KBytes */
            const char *kernel_filename,
            Stm32Gpio **stm32_gpio,
            const uint32_t *gpio_idr_masks,
            Stm32Uart **stm32_uart,
            Stm32Timer **stm32_timer,
            DeviceState **stm32_rtc,
            uint32_t osc_freq,
            uint32_t osc32_freq,
            struct stm32f4xx *stm,
            ARMCPU **cpu)
{
    MemoryRegion *address_space_mem = get_system_memory();
    DriveInfo *dinfo;
    DeviceState *nvic;
    int i;

    Object *stm32_container = container_get(qdev_get_machine(), "/stm32");

    nvic = armv7m_translated_init(
                stm32_container,          /* parent */
                address_space_mem,        /* address space memory */
                flash_size * 1024,        /* flash size in bytes */
                ram_size * 1024,          /* sram size in bytes */
                0,                        /* default number of irqs */
                kernel_filename,          /* kernel filename */
                kernel_load_translate_fn, /* kernel translate address function */
                NULL,                     /* translate  function opaque argument */
                "cortex-m4",              /* cpu model */
                cpu);                     /* Returned cpu instance */

    qdev_connect_gpio_out_named(nvic, "SYSRESETREQ", 0,
                                qemu_allocate_irq(&do_sys_reset, NULL, 0));


{
 MemoryRegion *hack = g_new(MemoryRegion, 1);



    /* Hack to map an additional page of ram at the top of the address
       space.  This stops qemu complaining about executing code outside RAM
       when returning from an exception.  */
    memory_region_init_ram(hack, NULL, "armv7m.hack", 0x1000, &error_fatal);
    vmstate_register_ram_global(hack);
    memory_region_add_subregion(system_memory, 0xfffff000, hack);

}

#endif

type_init(microbit_machine_init);
