
#ifndef _ASM_X86_USER_64_H
#define _ASM_X86_USER_64_H

#include <asm/types.h>
#include <asm/page.h>


struct user_i387_struct {
	unsigned short	cwd;
	unsigned short	swd;
	unsigned short	twd;	/* Note this is not the same as
				   the 32bit/x87/FSAVE twd */
	unsigned short	fop;
	__u64	rip;
	__u64	rdp;
	__u32	mxcsr;
	__u32	mxcsr_mask;
	__u32	st_space[32];	/* 8*16 bytes for each FP-reg = 128 bytes */
	__u32	xmm_space[64];	/* 16*16 bytes for each XMM-reg = 256 bytes */
	__u32	padding[24];
};

struct user_regs_struct {
	unsigned long	r15;
	unsigned long	r14;
	unsigned long	r13;
	unsigned long	r12;
	unsigned long	bp;
	unsigned long	bx;
	unsigned long	r11;
	unsigned long	r10;
	unsigned long	r9;
	unsigned long	r8;
	unsigned long	ax;
	unsigned long	cx;
	unsigned long	dx;
	unsigned long	si;
	unsigned long	di;
	unsigned long	orig_ax;
	unsigned long	ip;
	unsigned long	cs;
	unsigned long	flags;
	unsigned long	sp;
	unsigned long	ss;
	unsigned long	fs_base;
	unsigned long	gs_base;
	unsigned long	ds;
	unsigned long	es;
	unsigned long	fs;
	unsigned long	gs;
};


struct user {
  struct user_regs_struct regs;	/* Where the registers are actually stored */
/* ptrace does not yet supply these.  Someday.... */
  int u_fpvalid;		/* True if math co-processor being used. */
				/* for this mess. Not yet used. */
  int pad0;
  struct user_i387_struct i387;	/* Math Co-processor registers. */
/* The rest of this junk is to help gdb figure out what goes where */
  unsigned long int u_tsize;	/* Text segment size (pages). */
  unsigned long int u_dsize;	/* Data segment size (pages). */
  unsigned long int u_ssize;	/* Stack segment size (pages). */
  unsigned long start_code;     /* Starting virtual address of text. */
  unsigned long start_stack;	/* Starting virtual address of stack area.
				   This is actually the bottom of the stack,
				   the top of the stack is always found in the
				   esp register.  */
  long int signal;		/* Signal that caused the core dump. */
  int reserved;			/* No longer used */
  int pad1;
  unsigned long u_ar0;		/* Used by gdb to help find the values for */
				/* the registers. */
  struct user_i387_struct *u_fpstate;	/* Math Co-processor pointer. */
  unsigned long magic;		/* To uniquely identify a core file */
  char u_comm[32];		/* User command that was responsible */
  unsigned long u_debugreg[8];
  unsigned long error_code; /* CPU error code or 0 */
  unsigned long fault_address; /* CR3 or 0 */
};
#define NBPG PAGE_SIZE
#define UPAGES 1
#define HOST_TEXT_START_ADDR (u.start_code)
#define HOST_STACK_END_ADDR (u.start_stack + u.u_ssize * NBPG)

#endif /* _ASM_X86_USER_64_H */
