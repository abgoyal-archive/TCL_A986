
#include <asm/mipsregs.h>
#include <asm/page.h>
#include <asm/regdef.h>
#include <asm/stackframe.h>

	.macro tlb_do_page_fault, write
	NESTED(tlb_do_page_fault_\write, PT_SIZE, sp)
	SAVE_ALL
	MFC0	a2, CP0_BADVADDR
	KMODE
	move	a0, sp
	REG_S	a2, PT_BVADDR(sp)
	li	a1, \write
	PTR_LA	ra, ret_from_exception
	j	do_page_fault
	END(tlb_do_page_fault_\write)
	.endm

	tlb_do_page_fault 0
	tlb_do_page_fault 1
