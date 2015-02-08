
#ifndef _ASM_POWERPC_KPROBES_H
#define _ASM_POWERPC_KPROBES_H
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/percpu.h>

#define  __ARCH_WANT_KPROBES_INSN_SLOT

struct pt_regs;
struct kprobe;

typedef unsigned int kprobe_opcode_t;
#define BREAKPOINT_INSTRUCTION	0x7fe00008	/* trap */
#define MAX_INSN_SIZE 1

#define IS_TW(instr)		(((instr) & 0xfc0007fe) == 0x7c000008)
#define IS_TD(instr)		(((instr) & 0xfc0007fe) == 0x7c000088)
#define IS_TDI(instr)		(((instr) & 0xfc000000) == 0x08000000)
#define IS_TWI(instr)		(((instr) & 0xfc000000) == 0x0c000000)

#ifdef CONFIG_PPC64
#define kprobe_lookup_name(name, addr)					\
{									\
	addr = (kprobe_opcode_t *)kallsyms_lookup_name(name);		\
	if (addr) {							\
		char *colon;						\
		if ((colon = strchr(name, ':')) != NULL) {		\
			colon++;					\
			if (*colon != '\0' && *colon != '.')		\
				addr = *(kprobe_opcode_t **)addr;	\
		} else if (name[0] != '.')				\
			addr = *(kprobe_opcode_t **)addr;		\
	} else {							\
		char dot_name[KSYM_NAME_LEN];				\
		dot_name[0] = '.';					\
		dot_name[1] = '\0';					\
		strncat(dot_name, name, KSYM_NAME_LEN - 2);		\
		addr = (kprobe_opcode_t *)kallsyms_lookup_name(dot_name); \
	}								\
}

#define is_trap(instr)	(IS_TW(instr) || IS_TD(instr) || \
			IS_TWI(instr) || IS_TDI(instr))
#else
/* Use stock kprobe_lookup_name since ppc32 doesn't use function descriptors */
#define is_trap(instr)	(IS_TW(instr) || IS_TWI(instr))
#endif

#define flush_insn_slot(p)	do { } while (0)
#define kretprobe_blacklist_size 0

void kretprobe_trampoline(void);
extern void arch_remove_kprobe(struct kprobe *p);

/* Architecture specific copy of original instruction */
struct arch_specific_insn {
	/* copy of original instruction */
	kprobe_opcode_t *insn;
	/*
	 * Set in kprobes code, initially to 0. If the instruction can be
	 * eumulated, this is set to 1, if not, to -1.
	 */
	int boostable;
};

struct prev_kprobe {
	struct kprobe *kp;
	unsigned long status;
	unsigned long saved_msr;
};

/* per-cpu kprobe control block */
struct kprobe_ctlblk {
	unsigned long kprobe_status;
	unsigned long kprobe_saved_msr;
	struct pt_regs jprobe_saved_regs;
	struct prev_kprobe prev_kprobe;
};

extern int kprobe_exceptions_notify(struct notifier_block *self,
					unsigned long val, void *data);
extern int kprobe_fault_handler(struct pt_regs *regs, int trapnr);
#endif /* __KERNEL__ */
#endif	/* _ASM_POWERPC_KPROBES_H */
