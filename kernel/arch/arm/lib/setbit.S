
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "bitops.h"
		.text

ENTRY(_set_bit_be)
		eor	r0, r0, #0x18		@ big endian byte ordering
ENTRY(_set_bit_le)
	bitop	orr
ENDPROC(_set_bit_be)
ENDPROC(_set_bit_le)
