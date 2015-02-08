

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/platform_device.h>

#include <asm/system.h>
#include <asm/leds.h>
#include <asm/mach-types.h>

#include <asm/irq.h>
#include <mach/map.h>
#include <plat/regs-timer.h>
#include <mach/regs-irq.h>
#include <asm/mach/time.h>
#include <mach/tick.h>

#include <plat/clock.h>
#include <plat/cpu.h>

static unsigned long timer_startval;
static unsigned long timer_usec_ticks;

#ifndef TICK_MAX
#define TICK_MAX (0xffff)
#endif

#define TIMER_USEC_SHIFT 16




static inline unsigned long
timer_mask_usec_ticks(unsigned long scaler, unsigned long pclk)
{
	unsigned long den = pclk / 1000;

	return ((1000 << TIMER_USEC_SHIFT) * scaler + (den >> 1)) / den;
}


static inline unsigned long timer_ticks_to_usec(unsigned long ticks)
{
	unsigned long res;

	res = ticks * timer_usec_ticks;
	res += 1 << (TIMER_USEC_SHIFT - 4);	/* round up slightly */

	return res >> TIMER_USEC_SHIFT;
}


static unsigned long s3c2410_gettimeoffset (void)
{
	unsigned long tdone;
	unsigned long tval;

	/* work out how many ticks have gone since last timer interrupt */

	tval =  __raw_readl(S3C2410_TCNTO(4));
	tdone = timer_startval - tval;

	/* check to see if there is an interrupt pending */

	if (s3c24xx_ostimer_pending()) {
		/* re-read the timer, and try and fix up for the missed
		 * interrupt. Note, the interrupt may go off before the
		 * timer has re-loaded from wrapping.
		 */

		tval =  __raw_readl(S3C2410_TCNTO(4));
		tdone = timer_startval - tval;

		if (tval != 0)
			tdone += timer_startval;
	}

	return timer_ticks_to_usec(tdone);
}


static irqreturn_t
s3c2410_timer_interrupt(int irq, void *dev_id)
{
	timer_tick();
	return IRQ_HANDLED;
}

static struct irqaction s3c2410_timer_irq = {
	.name		= "S3C2410 Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= s3c2410_timer_interrupt,
};

#define use_tclk1_12() ( \
	machine_is_bast()	|| \
	machine_is_vr1000()	|| \
	machine_is_anubis()	|| \
	machine_is_osiris())

static struct clk *tin;
static struct clk *tdiv;
static struct clk *timerclk;

static void s3c2410_timer_setup (void)
{
	unsigned long tcon;
	unsigned long tcnt;
	unsigned long tcfg1;
	unsigned long tcfg0;

	tcnt = TICK_MAX;  /* default value for tcnt */

	/* configure the system for whichever machine is in use */

	if (use_tclk1_12()) {
		/* timer is at 12MHz, scaler is 1 */
		timer_usec_ticks = timer_mask_usec_ticks(1, 12000000);
		tcnt = 12000000 / HZ;

		tcfg1 = __raw_readl(S3C2410_TCFG1);
		tcfg1 &= ~S3C2410_TCFG1_MUX4_MASK;
		tcfg1 |= S3C2410_TCFG1_MUX4_TCLK1;
		__raw_writel(tcfg1, S3C2410_TCFG1);
	} else {
		unsigned long pclk;
		struct clk *tscaler;

		/* for the h1940 (and others), we use the pclk from the core
		 * to generate the timer values. since values around 50 to
		 * 70MHz are not values we can directly generate the timer
		 * value from, we need to pre-scale and divide before using it.
		 *
		 * for instance, using 50.7MHz and dividing by 6 gives 8.45MHz
		 * (8.45 ticks per usec)
		 */

		pclk = clk_get_rate(timerclk);

		/* configure clock tick */

		timer_usec_ticks = timer_mask_usec_ticks(6, pclk);

		tscaler = clk_get_parent(tdiv);

		clk_set_rate(tscaler, pclk / 3);
		clk_set_rate(tdiv, pclk / 6);
		clk_set_parent(tin, tdiv);

		tcnt = clk_get_rate(tin) / HZ;
	}

	tcon = __raw_readl(S3C2410_TCON);
	tcfg0 = __raw_readl(S3C2410_TCFG0);
	tcfg1 = __raw_readl(S3C2410_TCFG1);

	/* timers reload after counting zero, so reduce the count by 1 */

	tcnt--;

	printk(KERN_DEBUG "timer tcon=%08lx, tcnt %04lx, tcfg %08lx,%08lx, usec %08lx\n",
	       tcon, tcnt, tcfg0, tcfg1, timer_usec_ticks);

	/* check to see if timer is within 16bit range... */
	if (tcnt > TICK_MAX) {
		panic("setup_timer: HZ is too small, cannot configure timer!");
		return;
	}

	__raw_writel(tcfg1, S3C2410_TCFG1);
	__raw_writel(tcfg0, S3C2410_TCFG0);

	timer_startval = tcnt;
	__raw_writel(tcnt, S3C2410_TCNTB(4));

	/* ensure timer is stopped... */

	tcon &= ~(7<<20);
	tcon |= S3C2410_TCON_T4RELOAD;
	tcon |= S3C2410_TCON_T4MANUALUPD;

	__raw_writel(tcon, S3C2410_TCON);
	__raw_writel(tcnt, S3C2410_TCNTB(4));
	__raw_writel(tcnt, S3C2410_TCMPB(4));

	/* start the timer running */
	tcon |= S3C2410_TCON_T4START;
	tcon &= ~S3C2410_TCON_T4MANUALUPD;
	__raw_writel(tcon, S3C2410_TCON);
}

static void __init s3c2410_timer_resources(void)
{
	struct platform_device tmpdev;

	tmpdev.dev.bus = &platform_bus_type;
	tmpdev.id = 4;

	timerclk = clk_get(NULL, "timers");
	if (IS_ERR(timerclk))
		panic("failed to get clock for system timer");

	clk_enable(timerclk);

	if (!use_tclk1_12()) {
		tin = clk_get(&tmpdev.dev, "pwm-tin");
		if (IS_ERR(tin))
			panic("failed to get pwm-tin clock for system timer");

		tdiv = clk_get(&tmpdev.dev, "pwm-tdiv");
		if (IS_ERR(tdiv))
			panic("failed to get pwm-tdiv clock for system timer");
	}

	clk_enable(tin);
}

static void __init s3c2410_timer_init(void)
{
	s3c2410_timer_resources();
	s3c2410_timer_setup();
	setup_irq(IRQ_TIMER4, &s3c2410_timer_irq);
}

struct sys_timer s3c24xx_timer = {
	.init		= s3c2410_timer_init,
	.offset		= s3c2410_gettimeoffset,
	.resume		= s3c2410_timer_setup
};
