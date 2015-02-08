
#include <linux/module.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <asm/io.h>

void __outsl(unsigned long addr, const void *buffer, int count)
{
	const unsigned char *buf = buffer;
	unsigned long val;

	while (count--) {
		memcpy(&val, buf, 4);
		outl(val, addr);
		buf += 4;
	}
}
EXPORT_SYMBOL(__outsl);
