
#include <linux/linkage.h>
#include <asm/assembler.h>

		.text
		.align

.Liosl_warning:
		.ascii	"<4>insl/outsl not implemented, called from %08lX\0"
		.align

ENTRY(insl)
ENTRY(outsl)
		adr	r0, .Liosl_warning
		mov	r1, lr
		b	printk
