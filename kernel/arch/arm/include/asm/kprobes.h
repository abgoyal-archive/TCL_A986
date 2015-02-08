

#ifndef _ARM_KPROBES_H
#define _ARM_KPROBES_H

#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/percpu.h>

#define __ARCH_WANT_KPROBES_INSN_SLOT
#define MAX_INSN_SIZE			2
#define MAX_STACK_SIZE			64	/* 32 would probably be OK */

#define KPROBE_BREAKPOINT_INSTRUCTION	0xe7f001f8

#define regs_return_value(regs)		((regs)->ARM_r0)
#define flush_insn_slot(p)		do { } while (0)
#define kretprobe_blacklist_size	0

typedef u32 kprobe_opcode_t;

struct kprobe;
typedef void (kprobe_insn_handler_t)(struct kprobe *, struct pt_regs *);

/* Architecture specific copy of original instruction. */
struct arch_specific_insn {
	kprobe_opcode_t		*insn;
	kprobe_insn_handler_t	*insn_handler;
};

struct prev_kprobe {
	struct kprobe *kp;
	unsigned int status;
};

/* per-cpu kprobe control block */
struct kprobe_ctlblk {
	unsigned int kprobe_status;
	struct prev_kprobe prev_kprobe;
	struct pt_regs jprobe_saved_regs;
	char jprobes_stack[MAX_STACK_SIZE];
};

void arch_remove_kprobe(struct kprobe *);
void kretprobe_trampoline(void);

int kprobe_fault_handler(struct pt_regs *regs, unsigned int fsr);
int kprobe_exceptions_notify(struct notifier_block *self,
			     unsigned long val, void *data);

enum kprobe_insn {
	INSN_REJECTED,
	INSN_GOOD,
	INSN_GOOD_NO_SLOT
};

enum kprobe_insn arm_kprobe_decode_insn(kprobe_opcode_t,
					struct arch_specific_insn *);
void __init arm_kprobe_decode_init(void);

#endif /* _ARM_KPROBES_H */
