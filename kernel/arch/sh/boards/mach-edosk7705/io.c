

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/io.h>
#include <mach/edosk7705.h>
#include <asm/addrspace.h>

#define SMC_IOADDR	0xA2000000

/* Map the Ethernet addresses as if it is at 0x300 - 0x320 */
static unsigned long sh_edosk7705_isa_port2addr(unsigned long port)
{
	/*
	 * SMC91C96 registers are 4 byte aligned rather than the
	 * usual 2 byte!
	 */
	if (port >= 0x300 && port < 0x320)
		return SMC_IOADDR + ((port - 0x300) * 2);

	maybebadio(port);
	return port;
}

unsigned char sh_edosk7705_inb(unsigned long port)
{
	if (port >= 0x300 && port < 0x320 && port & 0x01)
		return __raw_readw(port - 1) >> 8;

	return __raw_readb(sh_edosk7705_isa_port2addr(port));
}

void sh_edosk7705_outb(unsigned char value, unsigned long port)
{
	if (port >= 0x300 && port < 0x320 && port & 0x01) {
		__raw_writew(((unsigned short)value << 8), port - 1);
		return;
	}

	__raw_writeb(value, sh_edosk7705_isa_port2addr(port));
}

void sh_edosk7705_insb(unsigned long port, void *addr, unsigned long count)
{
	unsigned char *p = addr;

	while (count--)
		*p++ = sh_edosk7705_inb(port);
}

void sh_edosk7705_outsb(unsigned long port, const void *addr, unsigned long count)
{
	unsigned char *p = (unsigned char *)addr;

	while (count--)
		sh_edosk7705_outb(*p++, port);
}
