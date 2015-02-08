

#include <asm/processor.h>
#include <asm/ppc_asm.h>
#include <asm/asm-offsets.h>
#include <asm/unistd.h>
#include <asm/vdso.h>

	.text
V_FUNCTION_BEGIN(__get_datapage)
  .cfi_startproc
	/* We don't want that exposed or overridable as we want other objects
	 * to be able to bl directly to here
	 */
	.protected __get_datapage
	.hidden __get_datapage

	mflr	r0
  .cfi_register lr,r0

	bcl	20,31,1f
	.global	__kernel_datapage_offset;
__kernel_datapage_offset:
	.long	0
1:
	mflr	r3
	mtlr	r0
	lwz	r0,0(r3)
	add	r3,r0,r3
	blr
  .cfi_endproc
V_FUNCTION_END(__get_datapage)

V_FUNCTION_BEGIN(__kernel_get_syscall_map)
  .cfi_startproc
	mflr	r12
  .cfi_register lr,r12
	mr	r4,r3
	bl	__get_datapage@local
	mtlr	r12
	addi	r3,r3,CFG_SYSCALL_MAP32
	cmpli	cr0,r4,0
	beqlr
	li	r0,__NR_syscalls
	stw	r0,0(r4)
	crclr	cr0*4+so
	blr
  .cfi_endproc
V_FUNCTION_END(__kernel_get_syscall_map)

V_FUNCTION_BEGIN(__kernel_get_tbfreq)
  .cfi_startproc
	mflr	r12
  .cfi_register lr,r12
	bl	__get_datapage@local
	lwz	r4,(CFG_TB_TICKS_PER_SEC + 4)(r3)
	lwz	r3,CFG_TB_TICKS_PER_SEC(r3)
	mtlr	r12
	crclr	cr0*4+so
	blr
  .cfi_endproc
V_FUNCTION_END(__kernel_get_tbfreq)
