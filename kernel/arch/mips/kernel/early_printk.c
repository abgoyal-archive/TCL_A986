
#include <linux/console.h>
#include <linux/init.h>

#include <asm/setup.h>

extern void prom_putchar(char);

static void __init
early_console_write(struct console *con, const char *s, unsigned n)
{
	while (n-- && *s) {
		if (*s == '\n')
			prom_putchar('\r');
		prom_putchar(*s);
		s++;
	}
}

static struct console early_console __initdata = {
	.name	= "early",
	.write	= early_console_write,
	.flags	= CON_PRINTBUFFER | CON_BOOT,
	.index	= -1
};

static int early_console_initialized __initdata;

void __init setup_early_printk(void)
{
	if (early_console_initialized)
		return;
	early_console_initialized = 1;

	register_console(&early_console);
}
