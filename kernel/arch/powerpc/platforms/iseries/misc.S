

#include <asm/processor.h>
#include <asm/asm-offsets.h>
#include <asm/ppc_asm.h>

	.text

/* Handle pending interrupts in interrupt context */
_GLOBAL(iseries_handle_interrupts)
	li	r0,0x5555
	sc
	blr
