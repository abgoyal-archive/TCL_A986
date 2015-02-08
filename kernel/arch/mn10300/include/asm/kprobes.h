
#ifndef _ASM_KPROBES_H
#define _ASM_KPROBES_H

#include <linux/types.h>
#include <linux/ptrace.h>

struct kprobe;

typedef unsigned char kprobe_opcode_t;
#define BREAKPOINT_INSTRUCTION	0xff
#define MAX_INSN_SIZE 8
#define MAX_STACK_SIZE 128

/* Architecture specific copy of original instruction */
struct arch_specific_insn {
	/*  copy of original instruction
	 */
	kprobe_opcode_t insn[MAX_INSN_SIZE];
};

extern const int kretprobe_blacklist_size;

extern int kprobe_exceptions_notify(struct notifier_block *self,
				    unsigned long val, void *data);

#define flush_insn_slot(p)  do {} while (0)

extern void arch_remove_kprobe(struct kprobe *p);

#endif /* _ASM_KPROBES_H */
