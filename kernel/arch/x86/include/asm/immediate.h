
#ifndef _ASM_X86_IMMEDIATE_H
#define _ASM_X86_IMMEDIATE_H


#include <asm/asm.h>

struct __imv {
	unsigned long var;	/* Pointer to the identifier variable of the
				 * immediate value
				 */
	unsigned long imv;	/*
				 * Pointer to the memory location of the
				 * immediate value within the instruction.
				 */
	unsigned char size;	/* Type size. */
	unsigned char insn_size;/* Instruction size. */
} __attribute__ ((packed));

#define imv_read(name)							\
	({								\
		__typeof__(name##__imv) value;				\
		BUILD_BUG_ON(sizeof(value) > 8);			\
		switch (sizeof(value)) {				\
		case 1:							\
			asm(".section __discard,\"\",@progbits\n\t"	\
				"1:\n\t"				\
				"mov $0,%0\n\t"				\
				"2:\n\t"				\
				".previous\n\t"				\
				".section __imv,\"aw\",@progbits\n\t"	\
				_ASM_PTR "%c1, (3f)-%c2\n\t"		\
				".byte %c2, (2b-1b)\n\t"		\
				".previous\n\t"				\
				"mov $0,%0\n\t"				\
				"3:\n\t"				\
				: "=q" (value)				\
				: "i" (&name##__imv),			\
				  "i" (sizeof(value)));			\
			break;						\
		case 2:							\
		case 4:							\
			asm(".section __discard,\"\",@progbits\n\t"	\
				"1:\n\t"				\
				"mov $0,%0\n\t"				\
				"2:\n\t"				\
				".previous\n\t"				\
				".section __imv,\"aw\",@progbits\n\t"	\
				_ASM_PTR "%c1, (3f)-%c2\n\t"		\
				".byte %c2, (2b-1b)\n\t"		\
				".previous\n\t"				\
				".org . + ((-.-(2b-1b)) & (%c2-1)), 0x90\n\t" \
				"mov $0,%0\n\t"				\
				"3:\n\t"				\
				: "=r" (value)				\
				: "i" (&name##__imv),			\
				  "i" (sizeof(value)));			\
			break;						\
		case 8:							\
			if (sizeof(long) < 8) {				\
				value = name##__imv;			\
				break;					\
			}						\
			asm(".section __discard,\"\",@progbits\n\t"	\
				"1:\n\t"				\
				"mov $0xFEFEFEFE01010101,%0\n\t"	\
				"2:\n\t"				\
				".previous\n\t"				\
				".section __imv,\"aw\",@progbits\n\t"	\
				_ASM_PTR "%c1, (3f)-%c2\n\t"		\
				".byte %c2, (2b-1b)\n\t"		\
				".previous\n\t"				\
				".org . + ((-.-(2b-1b)) & (%c2-1)), 0x90\n\t" \
				"mov $0xFEFEFEFE01010101,%0\n\t" 	\
				"3:\n\t"				\
				: "=r" (value)				\
				: "i" (&name##__imv),			\
				  "i" (sizeof(value)));			\
			break;						\
		};							\
		value;							\
	})

extern int arch_imv_update(const struct __imv *imv, int early);

#endif /* _ASM_X86_IMMEDIATE_H */
