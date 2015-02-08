#ifndef _LINUX_VM86_H
#define _LINUX_VM86_H


#define TF_MASK		0x00000100
#define IF_MASK		0x00000200
#define IOPL_MASK	0x00003000
#define NT_MASK		0x00004000
#ifdef CONFIG_VM86
#define VM_MASK		0x00020000
#else
#define VM_MASK		0 /* ignored */
#endif
#define AC_MASK		0x00040000
#define VIF_MASK	0x00080000	/* virtual interrupt flag */
#define VIP_MASK	0x00100000	/* virtual interrupt pending */
#define ID_MASK		0x00200000

#define BIOSSEG		0x0f000

#define CPU_086		0
#define CPU_186		1
#define CPU_286		2
#define CPU_386		3
#define CPU_486		4
#define CPU_586		5

#define VM86_TYPE(retval)	((retval) & 0xff)
#define VM86_ARG(retval)	((retval) >> 8)

#define VM86_SIGNAL	0	/* return due to signal */
#define VM86_UNKNOWN	1	/* unhandled GP fault - IO-instruction or similar */
#define VM86_INTx	2	/* int3/int x instruction (ARG = x) */
#define VM86_STI	3	/* sti/popf/iret instruction enabled virtual interrupts */

#define VM86_PICRETURN	4	/* return due to pending PIC request */
#define VM86_TRAP	6	/* return due to DOS-debugger request */

#define VM86_PLUS_INSTALL_CHECK	0
#define VM86_ENTER		1
#define VM86_ENTER_NO_BYPASS	2
#define	VM86_REQUEST_IRQ	3
#define VM86_FREE_IRQ		4
#define VM86_GET_IRQ_BITS	5
#define VM86_GET_AND_RESET_IRQ	6


struct vm86_regs {
	long ebx;
	long ecx;
	long edx;
	long esi;
	long edi;
	long ebp;
	long eax;
	long __null_ds;
	long __null_es;
	long __null_fs;
	long __null_gs;
	long orig_eax;
	long eip;
	unsigned short cs, __csh;
	long eflags;
	long esp;
	unsigned short ss, __ssh;
	unsigned short es, __esh;
	unsigned short ds, __dsh;
	unsigned short fs, __fsh;
	unsigned short gs, __gsh;
};

struct revectored_struct {
	unsigned long __map[8];			/* 256 bits */
};

struct vm86_struct {
	struct vm86_regs regs;
	unsigned long flags;
	unsigned long screen_bitmap;
	unsigned long cpu_type;
	struct revectored_struct int_revectored;
	struct revectored_struct int21_revectored;
};

#define VM86_SCREEN_BITMAP	0x0001

struct vm86plus_info_struct {
	unsigned long force_return_for_pic:1;
	unsigned long vm86dbg_active:1;       /* for debugger */
	unsigned long vm86dbg_TFpendig:1;     /* for debugger */
	unsigned long unused:28;
	unsigned long is_vm86pus:1;	      /* for vm86 internal use */
	unsigned char vm86dbg_intxxtab[32];   /* for debugger */
};

struct vm86plus_struct {
	struct vm86_regs regs;
	unsigned long flags;
	unsigned long screen_bitmap;
	unsigned long cpu_type;
	struct revectored_struct int_revectored;
	struct revectored_struct int21_revectored;
	struct vm86plus_info_struct vm86plus;
};

#ifdef __KERNEL__
#include <asm/ptrace.h>

struct kernel_vm86_regs {
	struct pt_regs pt;
	unsigned short es, __esh;
	unsigned short ds, __dsh;
	unsigned short fs, __fsh;
	unsigned short gs, __gsh;
};

struct kernel_vm86_struct {
	struct kernel_vm86_regs regs;
#define VM86_TSS_ESP0 flags
	unsigned long flags;
	unsigned long screen_bitmap;
	unsigned long cpu_type;
	struct revectored_struct int_revectored;
	struct revectored_struct int21_revectored;
	struct vm86plus_info_struct vm86plus;
	struct pt_regs *regs32;   /* here we save the pointer to the old regs */
};

#ifdef CONFIG_VM86

void handle_vm86_fault(struct kernel_vm86_regs *, long);
int handle_vm86_trap(struct kernel_vm86_regs *, long, int);

struct task_struct;
void release_vm86_irqs(struct task_struct *);

#else

#define handle_vm86_fault(a, b)
#define release_vm86_irqs(a)

static inline int handle_vm86_trap(struct kernel_vm86_regs *a, long b, int c) {
	return 0;
}

#endif /* CONFIG_VM86 */

#endif /* __KERNEL__ */

#endif
