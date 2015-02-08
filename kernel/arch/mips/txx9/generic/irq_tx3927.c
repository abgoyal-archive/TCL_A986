
#include <linux/init.h>
#include <asm/txx9irq.h>
#include <asm/txx9/tx3927.h>

void __init tx3927_irq_init(void)
{
	int i;

	txx9_irq_init(TX3927_IRC_REG);
	/* raise priority for timers, sio */
	for (i = 0; i < TX3927_NR_TMR; i++)
		txx9_irq_set_pri(TX3927_IR_TMR(i), 6);
	for (i = 0; i < TX3927_NR_SIO; i++)
		txx9_irq_set_pri(TX3927_IR_SIO(i), 7);
}
