
#ifndef __ASM_AVR32_SIGCONTEXT_H
#define __ASM_AVR32_SIGCONTEXT_H

struct sigcontext {
	unsigned long	oldmask;

	/* CPU registers */
	unsigned long	sr;
	unsigned long	pc;
	unsigned long	lr;
	unsigned long	sp;
	unsigned long	r12;
	unsigned long	r11;
	unsigned long	r10;
	unsigned long	r9;
	unsigned long	r8;
	unsigned long	r7;
	unsigned long	r6;
	unsigned long	r5;
	unsigned long	r4;
	unsigned long	r3;
	unsigned long	r2;
	unsigned long	r1;
	unsigned long	r0;
};

#endif /* __ASM_AVR32_SIGCONTEXT_H */
