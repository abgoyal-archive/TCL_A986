

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/irq.h>

#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>

#include <plat/cpu-freq.h>

#include <plat/clock.h>
#include <plat/cpu.h>
#include <plat/pll.h>

/* initalise all the clocks */

void __init_or_cpufreq s3c24xx_setup_clocks(unsigned long fclk,
					   unsigned long hclk,
					   unsigned long pclk)
{
	clk_upll.rate = s3c24xx_get_pll(__raw_readl(S3C2410_UPLLCON),
					clk_xtal.rate);

	clk_mpll.rate = fclk;
	clk_h.rate = hclk;
	clk_p.rate = pclk;
	clk_f.rate = fclk;
}
