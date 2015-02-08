
#ifndef __ASM_AVR32_KPROBES_H
#define __ASM_AVR32_KPROBES_H

#include <linux/types.h>

typedef u16	kprobe_opcode_t;
#define BREAKPOINT_INSTRUCTION	0xd673	/* breakpoint */
#define MAX_INSN_SIZE		2

#define kretprobe_blacklist_size 0

#define arch_remove_kprobe(p)	do { } while (0)

/* Architecture specific copy of original instruction */
struct arch_specific_insn {
	kprobe_opcode_t	insn[MAX_INSN_SIZE];
};

extern int kprobe_fault_handler(struct pt_regs *regs, int trapnr);
extern int kprobe_exceptions_notify(struct notifier_block *self,
				    unsigned long val, void *data);

#define flush_insn_slot(p)	do { } while (0)

#endif /* __ASM_AVR32_KPROBES_H */
