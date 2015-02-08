
#ifndef _ASM_X86_KPROBES_H
#define _ASM_X86_KPROBES_H
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/percpu.h>
#include <asm/insn.h>

#define  __ARCH_WANT_KPROBES_INSN_SLOT

struct pt_regs;
struct kprobe;

typedef u8 kprobe_opcode_t;
#define BREAKPOINT_INSTRUCTION	0xcc
#define RELATIVEJUMP_OPCODE 0xe9
#define RELATIVEJUMP_SIZE 5
#define RELATIVECALL_OPCODE 0xe8
#define RELATIVE_ADDR_SIZE 4
#define MAX_STACK_SIZE 64
#define MIN_STACK_SIZE(ADDR)					       \
	(((MAX_STACK_SIZE) < (((unsigned long)current_thread_info()) + \
			      THREAD_SIZE - (unsigned long)(ADDR)))    \
	 ? (MAX_STACK_SIZE)					       \
	 : (((unsigned long)current_thread_info()) +		       \
	    THREAD_SIZE - (unsigned long)(ADDR)))

#define flush_insn_slot(p)	do { } while (0)

/* optinsn template addresses */
extern kprobe_opcode_t optprobe_template_entry;
extern kprobe_opcode_t optprobe_template_val;
extern kprobe_opcode_t optprobe_template_call;
extern kprobe_opcode_t optprobe_template_end;
#define MAX_OPTIMIZED_LENGTH (MAX_INSN_SIZE + RELATIVE_ADDR_SIZE)
#define MAX_OPTINSN_SIZE 				\
	(((unsigned long)&optprobe_template_end -	\
	  (unsigned long)&optprobe_template_entry) +	\
	 MAX_OPTIMIZED_LENGTH + RELATIVEJUMP_SIZE)

extern const int kretprobe_blacklist_size;

void arch_remove_kprobe(struct kprobe *p);
void kretprobe_trampoline(void);

/* Architecture specific copy of original instruction*/
struct arch_specific_insn {
	/* copy of the original instruction */
	kprobe_opcode_t *insn;
	/*
	 * boostable = -1: This instruction type is not boostable.
	 * boostable = 0: This instruction type is boostable.
	 * boostable = 1: This instruction has been boosted: we have
	 * added a relative jump after the instruction copy in insn,
	 * so no single-step and fixup are needed (unless there's
	 * a post_handler or break_handler).
	 */
	int boostable;
};

struct arch_optimized_insn {
	/* copy of the original instructions */
	kprobe_opcode_t copied_insn[RELATIVE_ADDR_SIZE];
	/* detour code buffer */
	kprobe_opcode_t *insn;
	/* the size of instructions copied to detour code buffer */
	size_t size;
};

/* Return true (!0) if optinsn is prepared for optimization. */
static inline int arch_prepared_optinsn(struct arch_optimized_insn *optinsn)
{
	return optinsn->size;
}

struct prev_kprobe {
	struct kprobe *kp;
	unsigned long status;
	unsigned long old_flags;
	unsigned long saved_flags;
};

/* per-cpu kprobe control block */
struct kprobe_ctlblk {
	unsigned long kprobe_status;
	unsigned long kprobe_old_flags;
	unsigned long kprobe_saved_flags;
	unsigned long *jprobe_saved_sp;
	struct pt_regs jprobe_saved_regs;
	kprobe_opcode_t jprobes_stack[MAX_STACK_SIZE];
	struct prev_kprobe prev_kprobe;
};

extern int kprobe_fault_handler(struct pt_regs *regs, int trapnr);
extern int kprobe_exceptions_notify(struct notifier_block *self,
				    unsigned long val, void *data);
#endif /* _ASM_X86_KPROBES_H */
