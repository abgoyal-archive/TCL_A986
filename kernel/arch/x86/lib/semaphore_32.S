

#include <linux/linkage.h>
#include <asm/rwlock.h>
#include <asm/alternative-asm.h>
#include <asm/frame.h>
#include <asm/dwarf2.h>

	.section .sched.text, "ax"

#ifdef CONFIG_SMP
ENTRY(__write_lock_failed)
	CFI_STARTPROC simple
	FRAME
2: 	LOCK_PREFIX
	addl	$ RW_LOCK_BIAS,(%eax)
1:	rep; nop
	cmpl	$ RW_LOCK_BIAS,(%eax)
	jne	1b
	LOCK_PREFIX
	subl	$ RW_LOCK_BIAS,(%eax)
	jnz	2b
	ENDFRAME
	ret
	CFI_ENDPROC
	ENDPROC(__write_lock_failed)

ENTRY(__read_lock_failed)
	CFI_STARTPROC
	FRAME
2: 	LOCK_PREFIX
	incl	(%eax)
1:	rep; nop
	cmpl	$1,(%eax)
	js	1b
	LOCK_PREFIX
	decl	(%eax)
	js	2b
	ENDFRAME
	ret
	CFI_ENDPROC
	ENDPROC(__read_lock_failed)

#endif

#ifdef CONFIG_RWSEM_XCHGADD_ALGORITHM

/* Fix up special calling conventions */
ENTRY(call_rwsem_down_read_failed)
	CFI_STARTPROC
	push %ecx
	CFI_ADJUST_CFA_OFFSET 4
	CFI_REL_OFFSET ecx,0
	push %edx
	CFI_ADJUST_CFA_OFFSET 4
	CFI_REL_OFFSET edx,0
	call rwsem_down_read_failed
	pop %edx
	CFI_ADJUST_CFA_OFFSET -4
	pop %ecx
	CFI_ADJUST_CFA_OFFSET -4
	ret
	CFI_ENDPROC
	ENDPROC(call_rwsem_down_read_failed)

ENTRY(call_rwsem_down_write_failed)
	CFI_STARTPROC
	push %ecx
	CFI_ADJUST_CFA_OFFSET 4
	CFI_REL_OFFSET ecx,0
	calll rwsem_down_write_failed
	pop %ecx
	CFI_ADJUST_CFA_OFFSET -4
	ret
	CFI_ENDPROC
	ENDPROC(call_rwsem_down_write_failed)

ENTRY(call_rwsem_wake)
	CFI_STARTPROC
	decw %dx    /* do nothing if still outstanding active readers */
	jnz 1f
	push %ecx
	CFI_ADJUST_CFA_OFFSET 4
	CFI_REL_OFFSET ecx,0
	call rwsem_wake
	pop %ecx
	CFI_ADJUST_CFA_OFFSET -4
1:	ret
	CFI_ENDPROC
	ENDPROC(call_rwsem_wake)

/* Fix up special calling conventions */
ENTRY(call_rwsem_downgrade_wake)
	CFI_STARTPROC
	push %ecx
	CFI_ADJUST_CFA_OFFSET 4
	CFI_REL_OFFSET ecx,0
	push %edx
	CFI_ADJUST_CFA_OFFSET 4
	CFI_REL_OFFSET edx,0
	call rwsem_downgrade_wake
	pop %edx
	CFI_ADJUST_CFA_OFFSET -4
	pop %ecx
	CFI_ADJUST_CFA_OFFSET -4
	ret
	CFI_ENDPROC
	ENDPROC(call_rwsem_downgrade_wake)

#endif
