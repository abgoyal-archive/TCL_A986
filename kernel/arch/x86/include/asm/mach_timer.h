
#ifndef _ASM_X86_MACH_DEFAULT_MACH_TIMER_H
#define _ASM_X86_MACH_DEFAULT_MACH_TIMER_H

#define CALIBRATE_TIME_MSEC 30 /* 30 msecs */
#define CALIBRATE_LATCH	\
	((CLOCK_TICK_RATE * CALIBRATE_TIME_MSEC + 1000/2)/1000)

static inline void mach_prepare_counter(void)
{
       /* Set the Gate high, disable speaker */
	outb((inb(0x61) & ~0x02) | 0x01, 0x61);

	/*
	 * Now let's take care of CTC channel 2
	 *
	 * Set the Gate high, program CTC channel 2 for mode 0,
	 * (interrupt on terminal count mode), binary count,
	 * load 5 * LATCH count, (LSB and MSB) to begin countdown.
	 *
	 * Some devices need a delay here.
	 */
	outb(0xb0, 0x43);			/* binary, mode 0, LSB/MSB, Ch 2 */
	outb_p(CALIBRATE_LATCH & 0xff, 0x42);	/* LSB of count */
	outb_p(CALIBRATE_LATCH >> 8, 0x42);       /* MSB of count */
}

static inline void mach_countup(unsigned long *count_p)
{
	unsigned long count = 0;
	do {
		count++;
	} while ((inb_p(0x61) & 0x20) == 0);
	*count_p = count;
}

#endif /* _ASM_X86_MACH_DEFAULT_MACH_TIMER_H */
