

#ifndef __ASM_PLAT_UNCOMPRESS_H
#define __ASM_PLAT_UNCOMPRESS_H

#define UARTDBG_BASE		0x80070000
#define UART(c)			(((volatile unsigned *)UARTDBG_BASE)[c])

static void putc(char c)
{
	/* Wait for TX fifo empty */
	while ((UART(6) & (1<<7)) == 0)
		continue;

	/* Write byte */
	UART(0) = c;

	/* Wait for last bit to exit the UART */
	while (UART(6) & (1<<3))
		continue;
}

static void flush(void)
{
}

#define arch_decomp_setup()

#define arch_decomp_wdog()

#endif /* __ASM_PLAT_UNCOMPRESS_H */
