
#include <linux/kernel.h>
#include <asm/system.h>

unsigned int instruction_size(unsigned int insn)
{
	/* Look for the common cases */
	switch ((insn & 0xf00f)) {
	case 0x0000:	/* movi20 */
	case 0x0001:	/* movi20s */
	case 0x3001:	/* 32-bit mov/fmov/movu variants */
		return 4;
	}

	/* And the special cases.. */
	switch ((insn & 0xf08f)) {
	case 0x3009:	/* 32-bit b*.b bit operations */
		return 4;
	}

	return 2;
}
