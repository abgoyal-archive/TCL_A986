#ifndef _I386_USER_H
#define _I386_USER_H

#include <asm/page.h>


struct user_i387_struct {
	long	cwd;
	long	swd;
	long	twd;
	long	fip;
	long	fcs;
	long	foo;
	long	fos;
	long	st_space[20];	/* 8*10 bytes for each FP-reg = 80 bytes */
};

struct user_fxsr_struct {
	unsigned short	cwd;
	unsigned short	swd;
	unsigned short	twd;
	unsigned short	fop;
	long	fip;
	long	fcs;
	long	foo;
	long	fos;
	long	mxcsr;
	long	reserved;
	long	st_space[32];	/* 8*16 bytes for each FP-reg = 128 bytes */
	long	xmm_space[32];	/* 8*16 bytes for each XMM-reg = 128 bytes */
	long	padding[56];
};

struct user_regs_struct {
	long ebx, ecx, edx, esi, edi, ebp, eax;
	unsigned short ds, __ds, es, __es;
	unsigned short fs, __fs, gs, __gs;
	long orig_eax, eip;
	unsigned short cs, __cs;
	long eflags, esp;
	unsigned short ss, __ss;
};

struct user{
  struct user_regs_struct regs;		/* Where the registers are actually stored */
/* ptrace does not yet supply these.  Someday.... */
  int u_fpvalid;		/* True if math co-processor being used. */
                                /* for this mess. Not yet used. */
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
  long int signal;     		/* Signal that caused the core dump. */
  int reserved;			/* No longer used */
  struct user_pt_regs * u_ar0;	/* Used by gdb to help find the values for */
				/* the registers. */
  struct user_i387_struct* u_fpstate;	/* Math Co-processor pointer. */
  unsigned long magic;		/* To uniquely identify a core file */
  char u_comm[32];		/* User command that was responsible */
  int u_debugreg[8];
};
#define NBPG PAGE_SIZE
#define UPAGES 1
#define HOST_TEXT_START_ADDR (u.start_code)
#define HOST_STACK_END_ADDR (u.start_stack + u.u_ssize * NBPG)

#endif /* _I386_USER_H */
