

#include <linux/preempt.h>
#include <linux/smp.h>
#include <linux/notifier.h>
#include <linux/module.h>
#include <linux/immediate.h>
#include <linux/kdebug.h>
#include <linux/rcupdate.h>
#include <linux/kprobes.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/cacheflush.h>

#define BREAKPOINT_INSTRUCTION  0xcc
#define BREAKPOINT_INS_LEN	1
#define NR_NOPS			10

static unsigned long target_after_int3;	/* EIP of the target after the int3 */
static unsigned long bypass_eip;	/* EIP of the bypass. */
static unsigned long bypass_after_int3;	/* EIP after the end-of-bypass int3 */
static unsigned long after_imv;	/*
					 * EIP where to resume after the
					 * single-stepping.
					 */

static inline void _imv_bypass(unsigned long *bypassaddr,
	unsigned long *breaknextaddr)
{
		asm volatile("jmp 2f;\n\t"
				".align 16;\n\t"
				"0:\n\t"
				".space 10, 0x90;\n\t"
				"1:\n\t"
				"int3;\n\t"
				"2:\n\t"
				"mov $(0b),%0;\n\t"
				"mov $((1b)+1),%1;\n\t"
				: "=r" (*bypassaddr),
				  "=r" (*breaknextaddr));
}

static void imv_synchronize_core(void *info)
{
	smp_rmb();	/* read new instructions before continuing */
	sync_core();	/* use cpuid to stop speculative execution */
}

static int imv_notifier(struct notifier_block *nb,
	unsigned long val, void *data)
{
	enum die_val die_val = (enum die_val) val;
	struct die_args *args = data;

	if (!args->regs || user_mode_vm(args->regs))
		return NOTIFY_DONE;

	if (die_val == DIE_INT3) {
		if (args->regs->ip == target_after_int3) {
			preempt_disable();
			args->regs->ip = bypass_eip;
			return NOTIFY_STOP;
		} else if (args->regs->ip == bypass_after_int3) {
			args->regs->ip = after_imv;
			preempt_enable();
			return NOTIFY_STOP;
		}
	}
	return NOTIFY_DONE;
}

static struct notifier_block imv_notify = {
	.notifier_call = imv_notifier,
	.priority = 0x7fffffff,	/* we need to be notified first */
};

__kprobes int arch_imv_update(const struct __imv *imv, int early)
{
	int ret;
	unsigned char opcode_size = imv->insn_size - imv->size;
	unsigned long insn = imv->imv - opcode_size;
	unsigned long len;
	void *buffer;

#ifdef CONFIG_KPROBES
	/*
	 * Fail if a kprobe has been set on this instruction.
	 * (TODO: we could eventually do better and modify all the (possibly
	 * nested) kprobes for this site if kprobes had an API for this.
	 */
	if (unlikely(!early
			&& *(unsigned char *)insn == BREAKPOINT_INSTRUCTION)) {
		printk(KERN_WARNING "Immediate value in conflict with kprobe. "
				    "Variable at %p, "
				    "instruction at %p, size %hu\n",
				    (void *)imv->imv,
				    (void *)imv->var, imv->size);
		return -EBUSY;
	}
#endif

	/*
	 * If the variable and the instruction have the same value, there is
	 * nothing to do.
	 */
	switch (imv->size) {
	case 1:	if (*(uint8_t *)imv->imv
				== *(uint8_t *)imv->var)
			return 0;
		break;
	case 2:	if (*(uint16_t *)imv->imv
				== *(uint16_t *)imv->var)
			return 0;
		break;
	case 4:	if (*(uint32_t *)imv->imv
				== *(uint32_t *)imv->var)
			return 0;
		break;
#ifdef CONFIG_X86_64
	case 8:	if (*(uint64_t *)imv->imv
				== *(uint64_t *)imv->var)
			return 0;
		break;
#endif
	default:return -EINVAL;
	}

	if (!early) {
		/* bypass is 10 bytes long for x86_64 long */
		WARN_ON(imv->insn_size > 10);
		_imv_bypass(&bypass_eip, &bypass_after_int3);

		after_imv = imv->imv + imv->size;
		len = NR_NOPS - imv->insn_size;

		/* Allocate buffer to prepare new bypass */
		buffer = kmalloc(imv->insn_size + len, GFP_KERNEL);
		memcpy(buffer, (void *)insn, imv->insn_size);
		/*
		 * Fill the rest with nops.
		 */
		add_nops(buffer + imv->insn_size, len);
		text_poke((void *)bypass_eip, buffer, imv->insn_size + len);
		kfree(buffer);

		target_after_int3 = insn + BREAKPOINT_INS_LEN;
		/* register_die_notifier has memory barriers */
		register_die_notifier(&imv_notify);
		/* The breakpoint will single-step the bypass */
		text_poke((void *)insn,
			((unsigned char[]){BREAKPOINT_INSTRUCTION}), 1);
		/*
		 * Make sure the breakpoint is set before we continue (visible
		 * to other CPUs and interrupts).
		 */
		smp_wmb();
		/*
		 * Execute smp_rmb() and serializing instruction on each CPU.
		 */
		ret = on_each_cpu(imv_synchronize_core, NULL, 1);
		BUG_ON(ret != 0);

		text_poke((void *)(insn + opcode_size), (void *)imv->var,
				imv->size);
		/*
		 * Make sure the value can be seen from other CPUs and
		 * interrupts.
		 */
		smp_wmb();
		/*
		 * Execute smp_rmb() on each CPU.
		 */
		ret = on_each_cpu(imv_synchronize_core, NULL, 1);
		BUG_ON(ret != 0);
		text_poke((void *)insn, (unsigned char *)bypass_eip, 1);
		/*
		 * Wait for all int3 handlers to end (interrupts are disabled in
		 * int3). This CPU is clearly not in a int3 handler, because
		 * int3 handler is not preemptible and there cannot be any more
		 * int3 handler called for this site, because we placed the
		 * original instruction back.  synchronize_sched has memory
		 * barriers.
		 */
		synchronize_sched();
		unregister_die_notifier(&imv_notify);
		/* unregister_die_notifier has memory barriers */
	} else
		text_poke_early((void *)imv->imv, (void *)imv->var,
			imv->size);
	return 0;
}
