

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/sysdev.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <mach/hardware.h>
#include <asm/proc-fns.h>
#include <asm/irq.h>

#include <mach/reset.h>
#include <mach/idle.h>
#include <mach/regs-s3c2443-clock.h>

#include <plat/gpio-core.h>
#include <plat/gpio-cfg.h>
#include <plat/gpio-cfg-helpers.h>
#include <plat/s3c2416.h>
#include <plat/devs.h>
#include <plat/cpu.h>

#include <plat/iic-core.h>

static struct map_desc s3c2416_iodesc[] __initdata = {
	IODESC_ENT(WATCHDOG),
	IODESC_ENT(CLKPWR),
	IODESC_ENT(TIMER),
};

struct sysdev_class s3c2416_sysclass = {
	.name = "s3c2416-core",
};

static struct sys_device s3c2416_sysdev = {
	.cls		= &s3c2416_sysclass,
};

static void s3c2416_hard_reset(void)
{
	__raw_writel(S3C2443_SWRST_RESET, S3C2443_SWRST);
}

int __init s3c2416_init(void)
{
	printk(KERN_INFO "S3C2416: Initializing architecture\n");

	s3c24xx_reset_hook = s3c2416_hard_reset;
	/* s3c24xx_idle = s3c2416_idle;	*/

	/* change WDT IRQ number */
	s3c_device_wdt.resource[1].start = IRQ_S3C2443_WDT;
	s3c_device_wdt.resource[1].end   = IRQ_S3C2443_WDT;

	/* the i2c devices are directly compatible with s3c2440 */
	s3c_i2c0_setname("s3c2440-i2c");
	s3c_i2c1_setname("s3c2440-i2c");

	s3c_device_fb.name = "s3c2443-fb";

	return sysdev_register(&s3c2416_sysdev);
}

void __init s3c2416_init_uarts(struct s3c2410_uartcfg *cfg, int no)
{
	s3c24xx_init_uartdevs("s3c2440-uart", s3c2410_uart_resources, cfg, no);

	s3c_device_nand.name = "s3c2416-nand";
}


void __init s3c2416_map_io(void)
{
	s3c24xx_gpiocfg_default.set_pull = s3c_gpio_setpull_updown;
	s3c24xx_gpiocfg_default.get_pull = s3c_gpio_getpull_updown;

	iotable_init(s3c2416_iodesc, ARRAY_SIZE(s3c2416_iodesc));
}


static int __init s3c2416_core_init(void)
{
	return sysdev_class_register(&s3c2416_sysclass);
}

core_initcall(s3c2416_core_init);
