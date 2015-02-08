
#include <asm/leds.h>
#include <asm/hardware/clps7111.h>

extern void clps711x_setup_timer(void);

static irqreturn_t
p720t_timer_interrupt(int irq, void *dev_id)
{
	struct pt_regs *regs = get_irq_regs();
	do_leds();
	do_timer(1);
#ifndef CONFIG_SMP
	update_process_times(user_mode(regs));
#endif
	do_profile(regs);
	return IRQ_HANDLED;
}

void __init time_init(void)
{
	clps711x_setup_timer();
	timer_irq.handler = p720t_timer_interrupt;
	setup_irq(IRQ_TC2OI, &timer_irq);
}
