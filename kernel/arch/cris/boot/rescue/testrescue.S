

#define ASSEMBLER_MACROS_ONLY
#include <arch/sv_addr_ag.h>

	.text

	nop
	nop
	moveq	-1, $r2
	move.b	$r2, [R_PORT_PA_DIR]
	moveq	0, $r2
	move.b	$r2, [R_PORT_PA_DATA]

endless:
	nop
	ba	endless
	nop

