
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "bitops.h"
                .text

ENTRY(_clear_bit_be)
		eor	r0, r0, #0x18		@ big endian byte ordering
ENTRY(_clear_bit_le)
	bitop	bic
ENDPROC(_clear_bit_be)
ENDPROC(_clear_bit_le)
