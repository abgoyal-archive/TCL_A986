
#ifndef __ASM_SH_USER_H
#define __ASM_SH_USER_H

#include <asm/ptrace.h>
#include <asm/page.h>


#if defined(__SH5__) || defined(CONFIG_CPU_SH5)
struct user_fpu_struct {
	unsigned long fp_regs[32];
	unsigned int fpscr;
};
#else
struct user_fpu_struct {
	unsigned long fp_regs[16];
	unsigned long xfp_regs[16];
	unsigned long fpscr;
	unsigned long fpul;
};
#endif

struct user {
	struct pt_regs	regs;			/* entire machine state */
	struct user_fpu_struct fpu;	/* Math Co-processor registers  */
	int u_fpvalid;		/* True if math co-processor being used */
	size_t		u_tsize;		/* text size (pages) */
	size_t		u_dsize;		/* data size (pages) */
	size_t		u_ssize;		/* stack size (pages) */
	unsigned long	start_code;		/* text starting address */
	unsigned long	start_data;		/* data starting address */
	unsigned long	start_stack;		/* stack starting address */
	long int	signal;			/* signal causing core dump */
	unsigned long	u_ar0;			/* help gdb find registers */
	struct user_fpu_struct* u_fpstate;	/* Math Co-processor pointer */
	unsigned long	magic;			/* identifies a core file */
	char		u_comm[32];		/* user command name */
};

#define NBPG			PAGE_SIZE
#define UPAGES			1
#define HOST_TEXT_START_ADDR	(u.start_code)
#define HOST_DATA_START_ADDR	(u.start_data)
#define HOST_STACK_END_ADDR	(u.start_stack + u.u_ssize * NBPG)

#endif /* __ASM_SH_USER_H */
