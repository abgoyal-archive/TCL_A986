
#include <linux/init.h>
#include <asm/bootinfo.h>
#include <asm/txx9/generic.h>
#include <asm/txx9/rbtx4927.h>

void __init rbtx4927_prom_init(void)
{
	add_memory_region(0, tx4927_get_mem_size(), BOOT_MEM_RAM);
	txx9_sio_putchar_init(TX4927_SIO_REG(0) & 0xfffffffffULL);
}
