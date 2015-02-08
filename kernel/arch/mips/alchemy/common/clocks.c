

#include <linux/module.h>
#include <linux/spinlock.h>
#include <asm/time.h>
#include <asm/mach-au1x00/au1000.h>

#define AU1000_SRC_CLK	12000000

static unsigned int au1x00_clock; /*  Hz */
static unsigned long uart_baud_base;

void set_au1x00_speed(unsigned int new_freq)
{
	au1x00_clock = new_freq;
}

unsigned int get_au1x00_speed(void)
{
	return au1x00_clock;
}
EXPORT_SYMBOL(get_au1x00_speed);

unsigned long get_au1x00_uart_baud_base(void)
{
	return uart_baud_base;
}

void set_au1x00_uart_baud_base(unsigned long new_baud_base)
{
	uart_baud_base = new_baud_base;
}

unsigned long au1xxx_calc_clock(void)
{
	unsigned long cpu_speed;

	/*
	 * On early Au1000, sys_cpupll was write-only. Since these
	 * silicon versions of Au1000 are not sold by AMD, we don't bend
	 * over backwards trying to determine the frequency.
	 */
	if (au1xxx_cpu_has_pll_wo())
#ifdef CONFIG_SOC_AU1000_FREQUENCY
		cpu_speed = CONFIG_SOC_AU1000_FREQUENCY;
#else
		cpu_speed = 396000000;
#endif
	else
		cpu_speed = (au_readl(SYS_CPUPLL) & 0x0000003f) * AU1000_SRC_CLK;

	/* On Alchemy CPU:counter ratio is 1:1 */
	mips_hpt_frequency = cpu_speed;
	/* Equation: Baudrate = CPU / (SD * 2 * CLKDIV * 16) */
	set_au1x00_uart_baud_base(cpu_speed / (2 * ((int)(au_readl(SYS_POWERCTRL)
							  & 0x03) + 2) * 16));

	set_au1x00_speed(cpu_speed);

	return cpu_speed;
}
