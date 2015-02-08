
#ifndef _M68K_USER_H
#define _M68K_USER_H


struct user_m68kfp_struct {
	unsigned long  fpregs[8*3];	/* fp0-fp7 registers */
	unsigned long  fpcntl[3];	/* fp control regs */
};

struct user_regs_struct {
	long d1,d2,d3,d4,d5,d6,d7;
	long a0,a1,a2,a3,a4,a5,a6;
	long d0;
	long usp;
	long orig_d0;
	short stkadj;
	short sr;
	long pc;
	short fmtvec;
	short __fill;
};


struct user{
  struct user_regs_struct regs;	/* Where the registers are actually stored */
/* ptrace does not yet supply these.  Someday.... */
  int u_fpvalid;		/* True if math co-processor being used. */
                                /* for this mess. Not yet used. */
  struct user_m68kfp_struct m68kfp; /* Math Co-processor registers. */
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
  unsigned long u_ar0;		/* Used by gdb to help find the values for */
				/* the registers. */
  struct user_m68kfp_struct* u_fpstate;	/* Math Co-processor pointer. */
  unsigned long magic;		/* To uniquely identify a core file */
  char u_comm[32];		/* User command that was responsible */
};
#define NBPG 4096
#define UPAGES 1
#define HOST_TEXT_START_ADDR (u.start_code)
#define HOST_STACK_END_ADDR (u.start_stack + u.u_ssize * NBPG)

#endif
