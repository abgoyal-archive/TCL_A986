
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <asm/div64.h>

void __delay(unsigned long loops)
{
	int d0;

	asm volatile(
		"	bra	1f	\n"
		"	.align	4	\n"
		"1:	bra	2f	\n"
		"	.align	4	\n"
		"2:	add	-1,%0	\n"
		"	bne	2b	\n"
		: "=&d" (d0)
		: "0" (loops)
		: "cc");
}
EXPORT_SYMBOL(__delay);

void __udelay(unsigned long usecs)
{
	signed long ioclk, stop;

	/* usecs * CLK / 1E6 */
	stop = __muldiv64u(usecs, MN10300_TSCCLK, 1000000);
	stop = TMTSCBC - stop;

	do {
		ioclk = TMTSCBC;
	} while (stop < ioclk);
}
EXPORT_SYMBOL(__udelay);
