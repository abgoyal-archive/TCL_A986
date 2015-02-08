
#ifndef _ASM_POWERPC_IMMEDIATE_H
#define _ASM_POWERPC_IMMEDIATE_H


#include <asm/asm-compat.h>

struct __imv {
	unsigned long var;	/* Identifier variable of the immediate value */
	unsigned long imv;	/*
				 * Pointer to the memory location that holds
				 * the immediate value within the load immediate
				 * instruction.
				 */
	unsigned char size;	/* Type size. */
} __attribute__ ((packed));

#define imv_read(name)							\
	({								\
		__typeof__(name##__imv) value;				\
		BUILD_BUG_ON(sizeof(value) > 8);			\
		switch (sizeof(value)) {				\
		case 1:							\
			asm(".section __imv,\"aw\",@progbits\n\t"	\
					PPC_LONG "%c1, ((1f)-1)\n\t"	\
					".byte 1\n\t"			\
					".previous\n\t"			\
					"li %0,0\n\t"			\
					"1:\n\t"			\
				: "=r" (value)				\
				: "i" (&name##__imv));			\
			break;						\
		case 2:							\
			asm(".section __imv,\"aw\",@progbits\n\t"	\
					PPC_LONG "%c1, ((1f)-2)\n\t"	\
					".byte 2\n\t"			\
					".previous\n\t"			\
					".align 2\n\t"			\
					"li %0,0\n\t"			\
					"1:\n\t"			\
				: "=r" (value)				\
				: "i" (&name##__imv));			\
			break;						\
		case 4:							\
		case 8:	value = name##__imv;				\
			break;						\
		};							\
		value;							\
	})

extern int arch_imv_update(const struct __imv *imv, int early);

#endif /* _ASM_POWERPC_IMMEDIATE_H */
