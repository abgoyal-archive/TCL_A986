
#ifndef _ASM_S390_KPROBES_H
#define _ASM_S390_KPROBES_H
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/percpu.h>

#define  __ARCH_WANT_KPROBES_INSN_SLOT
struct pt_regs;
struct kprobe;

typedef u16 kprobe_opcode_t;
#define BREAKPOINT_INSTRUCTION	0x0002

/* Maximum instruction size is 3 (16bit) halfwords: */
#define MAX_INSN_SIZE		0x0003
#define MAX_STACK_SIZE		64
#define MIN_STACK_SIZE(ADDR) (((MAX_STACK_SIZE) < \
	(((unsigned long)current_thread_info()) + THREAD_SIZE - (ADDR))) \
	? (MAX_STACK_SIZE) \
	: (((unsigned long)current_thread_info()) + THREAD_SIZE - (ADDR)))

#define kretprobe_blacklist_size 0

#define KPROBE_SWAP_INST	0x10

#define FIXUP_PSW_NORMAL	0x08
#define FIXUP_BRANCH_NOT_TAKEN	0x04
#define FIXUP_RETURN_REGISTER	0x02
#define FIXUP_NOT_REQUIRED	0x01

/* Architecture specific copy of original instruction */
struct arch_specific_insn {
	/* copy of original instruction */
	kprobe_opcode_t *insn;
	int fixup;
	int ilen;
	int reg;
};

struct ins_replace_args {
	kprobe_opcode_t *ptr;
	kprobe_opcode_t old;
	kprobe_opcode_t new;
};
struct prev_kprobe {
	struct kprobe *kp;
	unsigned long status;
	unsigned long saved_psw;
	unsigned long kprobe_saved_imask;
	unsigned long kprobe_saved_ctl[3];
};

/* per-cpu kprobe control block */
struct kprobe_ctlblk {
	unsigned long kprobe_status;
	unsigned long kprobe_saved_imask;
	unsigned long kprobe_saved_ctl[3];
	struct pt_regs jprobe_saved_regs;
	unsigned long jprobe_saved_r14;
	unsigned long jprobe_saved_r15;
	struct prev_kprobe prev_kprobe;
	kprobe_opcode_t jprobes_stack[MAX_STACK_SIZE];
};

void arch_remove_kprobe(struct kprobe *p);
void kretprobe_trampoline(void);
int  is_prohibited_opcode(kprobe_opcode_t *instruction);
void get_instruction_type(struct arch_specific_insn *ainsn);

int kprobe_fault_handler(struct pt_regs *regs, int trapnr);
int kprobe_exceptions_notify(struct notifier_block *self,
	unsigned long val, void *data);

#define flush_insn_slot(p)	do { } while (0)

#endif	/* _ASM_S390_KPROBES_H */
