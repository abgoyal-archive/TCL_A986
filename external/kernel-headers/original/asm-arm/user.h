#ifndef _ARM_USER_H
#define _ARM_USER_H

#include <asm/page.h>
#include <asm/ptrace.h>

struct user_fp {
	struct fp_reg {
		unsigned int sign1:1;
		unsigned int unused:15;
		unsigned int sign2:1;
		unsigned int exponent:14;
		unsigned int j:1;
		unsigned int mantissa1:31;
		unsigned int mantissa0:32;
	} fpregs[8];
	unsigned int fpsr:32;
	unsigned int fpcr:32;
	unsigned char ftype[8];
	unsigned int init_flag;
};

struct user{
  struct pt_regs regs;		/* Where the registers are actually stored */
/* ptrace does not yet supply these.  Someday.... */
  int u_fpvalid;		/* True if math co-processor being used. */
                                /* for this mess. Not yet used. */
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
  struct pt_regs * u_ar0;	/* Used by gdb to help find the values for */
				/* the registers. */
  unsigned long magic;		/* To uniquely identify a core file */
  char u_comm[32];		/* User command that was responsible */
  int u_debugreg[8];
  struct user_fp u_fp;		/* FP state */
  struct user_fp_struct * u_fp0;/* Used by gdb to help find the values for */
  				/* the FP registers. */
};
#define NBPG PAGE_SIZE
#define UPAGES 1
#define HOST_TEXT_START_ADDR (u.start_code)
#define HOST_STACK_END_ADDR (u.start_stack + u.u_ssize * NBPG)

#endif /* _ARM_USER_H */
