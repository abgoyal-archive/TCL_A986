
#include <linux/linkage.h>
#include <asm/assembler.h>
#include "bitops.h"
                .text

ENTRY(_test_and_set_bit_be)
		eor	r0, r0, #0x18		@ big endian byte ordering
ENTRY(_test_and_set_bit_le)
	testop	orreq, streqb
ENDPROC(_test_and_set_bit_be)
ENDPROC(_test_and_set_bit_le)
