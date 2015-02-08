
#include <linux/linkage.h>
#include <linux/lguest.h>
#include <asm/lguest_hcall.h>
#include <asm/asm-offsets.h>
#include <asm/thread_info.h>
#include <asm/processor-flags.h>

.section .init.text, "ax", @progbits
ENTRY(lguest_entry)
	/*
	 * We make the "initialization" hypercall now to tell the Host about
	 * us, and also find out where it put our page tables.
	 */
	movl $LHCALL_LGUEST_INIT, %eax
	movl $lguest_data - __PAGE_OFFSET, %ebx
	int $LGUEST_TRAP_ENTRY

	/* Set up the initial stack so we can run C code. */
	movl $(init_thread_union+THREAD_SIZE),%esp

	/* Jumps are relative: we're running __PAGE_OFFSET too low. */
	jmp lguest_init+__PAGE_OFFSET

.text
#define LGUEST_PATCH(name, insns...)			\
	lgstart_##name:	insns; lgend_##name:;		\
	.globl lgstart_##name; .globl lgend_##name

LGUEST_PATCH(cli, movl $0, lguest_data+LGUEST_DATA_irq_enabled)
LGUEST_PATCH(pushf, movl lguest_data+LGUEST_DATA_irq_enabled, %eax)

ENTRY(lg_irq_enable)
	/*
	 * The reverse of irq_disable, this sets lguest_data.irq_enabled to
	 * X86_EFLAGS_IF (ie. "Interrupts enabled").
	 */
	movl $X86_EFLAGS_IF, lguest_data+LGUEST_DATA_irq_enabled
	/*
	 * But now we need to check if the Host wants to know: there might have
	 * been interrupts waiting to be delivered, in which case it will have
	 * set lguest_data.irq_pending to X86_EFLAGS_IF.  If it's not zero, we
	 * jump to send_interrupts, otherwise we're done.
	 */
	testl $0, lguest_data+LGUEST_DATA_irq_pending
	jnz send_interrupts
	/*
	 * One cool thing about x86 is that you can do many things without using
	 * a register.  In this case, the normal path hasn't needed to save or
	 * restore any registers at all!
	 */
	ret
send_interrupts:
	/*
	 * OK, now we need a register: eax is used for the hypercall number,
	 * which is LHCALL_SEND_INTERRUPTS.
	 *
	 * We used not to bother with this pending detection at all, which was
	 * much simpler.  Sooner or later the Host would realize it had to
	 * send us an interrupt.  But that turns out to make performance 7
	 * times worse on a simple tcp benchmark.  So now we do this the hard
	 * way.
	 */
	pushl %eax
	movl $LHCALL_SEND_INTERRUPTS, %eax
	/*
	 * This is a vmcall instruction (same thing that KVM uses).  Older
	 * assembler versions might not know the "vmcall" instruction, so we
	 * create one manually here.
	 */
	.byte 0x0f,0x01,0xc1 /* KVM_HYPERCALL */
	/* Put eax back the way we found it. */
	popl %eax
	ret

ENTRY(lg_restore_fl)
	/* This is just "lguest_data.irq_enabled = flags;" */
	movl %eax, lguest_data+LGUEST_DATA_irq_enabled
	/*
	 * Now, if the %eax value has enabled interrupts and
	 * lguest_data.irq_pending is set, we want to tell the Host so it can
	 * deliver any outstanding interrupts.  Fortunately, both values will
	 * be X86_EFLAGS_IF (ie. 512) in that case, and the "testl"
	 * instruction will AND them together for us.  If both are set, we
	 * jump to send_interrupts.
	 */
	testl lguest_data+LGUEST_DATA_irq_pending, %eax
	jnz send_interrupts
	/* Again, the normal path has used no extra registers.  Clever, huh? */
	ret
/*:*/

/* These demark the EIP range where host should never deliver interrupts. */
.global lguest_noirq_start
.global lguest_noirq_end


ENTRY(lguest_iret)
	pushl	%eax
	movl	12(%esp), %eax
lguest_noirq_start:
	/*
	 * Note the %ss: segment prefix here.  Normal data accesses use the
	 * "ds" segment, but that will have already been restored for whatever
	 * we're returning to (such as userspace): we can't trust it.  The %ss:
	 * prefix makes sure we use the stack segment, which is still valid.
	 */
	movl	%eax,%ss:lguest_data+LGUEST_DATA_irq_enabled
	popl	%eax
	iret
lguest_noirq_end:
