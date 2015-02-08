
#ifndef _ASM_ARCH_GPTIMERS_H
#define _ASM_ARCH_GPTIMERS_H

#include <mach/hardware.h>

struct GPT_Regs {
	unsigned int TIMERLOAD;
	unsigned int TIMERVALUE;
	unsigned int TIMERCONTROL;
	unsigned int TIMERCLEAR;
};

#define GPT_BASE		(IO_BASE_2 + 0x3000)
#define l7200_timer1_regs	((volatile struct GPT_Regs *) (GPT_BASE))
#define l7200_timer2_regs	((volatile struct GPT_Regs *) (GPT_BASE + 0x20))

#define	GPT_PRESCALE_1		0x00000000
#define	GPT_PRESCALE_16		0x00000004
#define	GPT_PRESCALE_256	0x00000008
#define GPT_MODE_FREERUN	0x00000000
#define GPT_MODE_PERIODIC	0x00000040
#define GPT_ENABLE		0x00000080
#define GPT_BZTOG		0x00000100
#define GPT_BZMOD		0x00000200
#define GPT_LOAD_MASK 		0x0000ffff

#endif
