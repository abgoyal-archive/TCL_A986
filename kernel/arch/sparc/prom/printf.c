

#include <linux/kernel.h>
#include <linux/compiler.h>

#include <asm/openprom.h>
#include <asm/oplib.h>

static char ppbuf[1024];

void notrace prom_write(const char *buf, unsigned int n)
{
	char ch;

	while (n != 0) {
		--n;
		if ((ch = *buf++) == '\n')
			prom_putchar('\r');
		prom_putchar(ch);
	}
}

void notrace prom_printf(const char *fmt, ...)
{
	va_list args;
	int i;

	va_start(args, fmt);
	i = vscnprintf(ppbuf, sizeof(ppbuf), fmt, args);
	va_end(args);

	prom_write(ppbuf, i);
}
