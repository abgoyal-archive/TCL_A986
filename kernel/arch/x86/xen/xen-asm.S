

#include <asm/asm-offsets.h>
#include <asm/percpu.h>
#include <asm/processor-flags.h>

#include "xen-asm.h"

ENTRY(xen_irq_enable_direct)
	/* Unmask events */
	movb $0, PER_CPU_VAR(xen_vcpu_info) + XEN_vcpu_info_mask

	/*
	 * Preempt here doesn't matter because that will deal with any
	 * pending interrupts.  The pending check may end up being run
	 * on the wrong CPU, but that doesn't hurt.
	 */

	/* Test for pending */
	testb $0xff, PER_CPU_VAR(xen_vcpu_info) + XEN_vcpu_info_pending
	jz 1f

2:	call check_events
1:
ENDPATCH(xen_irq_enable_direct)
	ret
	ENDPROC(xen_irq_enable_direct)
	RELOC(xen_irq_enable_direct, 2b+1)


ENTRY(xen_irq_disable_direct)
	movb $1, PER_CPU_VAR(xen_vcpu_info) + XEN_vcpu_info_mask
ENDPATCH(xen_irq_disable_direct)
	ret
	ENDPROC(xen_irq_disable_direct)
	RELOC(xen_irq_disable_direct, 0)

ENTRY(xen_save_fl_direct)
	testb $0xff, PER_CPU_VAR(xen_vcpu_info) + XEN_vcpu_info_mask
	setz %ah
	addb %ah, %ah
ENDPATCH(xen_save_fl_direct)
	ret
	ENDPROC(xen_save_fl_direct)
	RELOC(xen_save_fl_direct, 0)


ENTRY(xen_restore_fl_direct)
#ifdef CONFIG_X86_64
	testw $X86_EFLAGS_IF, %di
#else
	testb $X86_EFLAGS_IF>>8, %ah
#endif
	setz PER_CPU_VAR(xen_vcpu_info) + XEN_vcpu_info_mask
	/*
	 * Preempt here doesn't matter because that will deal with any
	 * pending interrupts.  The pending check may end up being run
	 * on the wrong CPU, but that doesn't hurt.
	 */

	/* check for unmasked and pending */
	cmpw $0x0001, PER_CPU_VAR(xen_vcpu_info) + XEN_vcpu_info_pending
	jz 1f
2:	call check_events
1:
ENDPATCH(xen_restore_fl_direct)
	ret
	ENDPROC(xen_restore_fl_direct)
	RELOC(xen_restore_fl_direct, 2b+1)


check_events:
#ifdef CONFIG_X86_32
	push %eax
	push %ecx
	push %edx
	call xen_force_evtchn_callback
	pop %edx
	pop %ecx
	pop %eax
#else
	push %rax
	push %rcx
	push %rdx
	push %rsi
	push %rdi
	push %r8
	push %r9
	push %r10
	push %r11
	call xen_force_evtchn_callback
	pop %r11
	pop %r10
	pop %r9
	pop %r8
	pop %rdi
	pop %rsi
	pop %rdx
	pop %rcx
	pop %rax
#endif
	ret
