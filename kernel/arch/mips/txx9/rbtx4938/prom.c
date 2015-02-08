

#include <linux/init.h>
#include <linux/bootmem.h>
#include <asm/bootinfo.h>
#include <asm/txx9/generic.h>
#include <asm/txx9/rbtx4938.h>

void __init rbtx4938_prom_init(void)
{
	add_memory_region(0, tx4938_get_mem_size(), BOOT_MEM_RAM);
	txx9_sio_putchar_init(TX4938_SIO_REG(0) & 0xfffffffffULL);
}
