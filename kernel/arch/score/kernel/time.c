

#include <linux/clockchips.h>
#include <linux/interrupt.h>

#include <asm/scoreregs.h>

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evdev = dev_id;

	/* clear timer interrupt flag */
	outl(1, P_TIMER0_CPP_REG);
	evdev->event_handler(evdev);

	return IRQ_HANDLED;
}

static struct irqaction timer_irq = {
	.handler = timer_interrupt,
	.flags = IRQF_DISABLED | IRQF_TIMER,
	.name = "timer",
};

static int score_timer_set_next_event(unsigned long delta,
		struct clock_event_device *evdev)
{
	outl((TMR_M_PERIODIC | TMR_IE_ENABLE), P_TIMER0_CTRL);
	outl(delta, P_TIMER0_PRELOAD);
	outl(inl(P_TIMER0_CTRL) | TMR_ENABLE, P_TIMER0_CTRL);

	return 0;
}

static void score_timer_set_mode(enum clock_event_mode mode,
		struct clock_event_device *evdev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		outl((TMR_M_PERIODIC | TMR_IE_ENABLE), P_TIMER0_CTRL);
		outl(SYSTEM_CLOCK/HZ, P_TIMER0_PRELOAD);
		outl(inl(P_TIMER0_CTRL) | TMR_ENABLE, P_TIMER0_CTRL);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_UNUSED:
		break;
	default:
		BUG();
	}
}

static struct clock_event_device score_clockevent = {
	.name		= "score_clockevent",
	.features	= CLOCK_EVT_FEAT_PERIODIC,
	.shift		= 16,
	.set_next_event	= score_timer_set_next_event,
	.set_mode	= score_timer_set_mode,
};

void __init time_init(void)
{
	timer_irq.dev_id = &score_clockevent;
	setup_irq(IRQ_TIMER , &timer_irq);

	/* setup COMPARE clockevent */
	score_clockevent.mult = div_sc(SYSTEM_CLOCK, NSEC_PER_SEC,
					score_clockevent.shift);
	score_clockevent.max_delta_ns = clockevent_delta2ns((u32)~0,
					&score_clockevent);
	score_clockevent.min_delta_ns = clockevent_delta2ns(50,
						&score_clockevent) + 1;
	score_clockevent.cpumask = cpumask_of(0);
	clockevents_register_device(&score_clockevent);
}
