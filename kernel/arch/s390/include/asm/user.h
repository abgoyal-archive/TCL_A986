

#ifndef _S390_USER_H
#define _S390_USER_H

#include <asm/page.h>
#include <asm/ptrace.h>



struct user {
  struct user_regs_struct regs;		/* Where the registers are actually stored */
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
  unsigned long u_ar0;		/* Used by gdb to help find the values for */
				/* the registers. */
  unsigned long magic;		/* To uniquely identify a core file */
  char u_comm[32];		/* User command that was responsible */
};
#define NBPG PAGE_SIZE
#define UPAGES 1
#define HOST_TEXT_START_ADDR (u.start_code)
#define HOST_STACK_END_ADDR (u.start_stack + u.u_ssize * NBPG)

#endif /* _S390_USER_H */
