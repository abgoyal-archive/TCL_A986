
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/addrspace.h>

#ifdef CONFIG_SH_SECUREEDGE5410
unsigned short secureedge5410_ioport;
#endif

static inline volatile __u16 *port2adr(unsigned int port)
{
	maybebadio((unsigned long)port);
	return (volatile __u16*)port;
}

unsigned char snapgear_inb(unsigned long port)
{
	if (PXSEG(port))
		return *(volatile unsigned char *)port;
	else
		return (*port2adr(port)) & 0xff;
}

unsigned char snapgear_inb_p(unsigned long port)
{
	unsigned char v;

	if (PXSEG(port))
		v = *(volatile unsigned char *)port;
	else
		v = (*port2adr(port))&0xff;
	ctrl_delay();
	return v;
}

unsigned short snapgear_inw(unsigned long port)
{
	if (PXSEG(port))
		return *(volatile unsigned short *)port;
	else if (port >= 0x2000)
		return *port2adr(port);
	else
		maybebadio(port);
	return 0;
}

unsigned int snapgear_inl(unsigned long port)
{
	if (PXSEG(port))
		return *(volatile unsigned long *)port;
	else if (port >= 0x2000)
		return *port2adr(port);
	else
		maybebadio(port);
	return 0;
}

void snapgear_outb(unsigned char value, unsigned long port)
{

	if (PXSEG(port))
		*(volatile unsigned char *)port = value;
	else
		*(port2adr(port)) = value;
}

void snapgear_outb_p(unsigned char value, unsigned long port)
{
	if (PXSEG(port))
		*(volatile unsigned char *)port = value;
	else
		*(port2adr(port)) = value;
	ctrl_delay();
}

void snapgear_outw(unsigned short value, unsigned long port)
{
	if (PXSEG(port))
		*(volatile unsigned short *)port = value;
	else if (port >= 0x2000)
		*port2adr(port) = value;
	else
		maybebadio(port);
}

void snapgear_outl(unsigned int value, unsigned long port)
{
	if (PXSEG(port))
		*(volatile unsigned long *)port = value;
	else
		maybebadio(port);
}

void snapgear_insl(unsigned long port, void *addr, unsigned long count)
{
	maybebadio(port);
}

void snapgear_outsl(unsigned long port, const void *addr, unsigned long count)
{
	maybebadio(port);
}
