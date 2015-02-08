
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "bitops.h"
                .text

ENTRY(_change_bit_be)
		eor	r0, r0, #0x18		@ big endian byte ordering
ENTRY(_change_bit_le)
	bitop	eor
ENDPROC(_change_bit_be)
ENDPROC(_change_bit_le)
