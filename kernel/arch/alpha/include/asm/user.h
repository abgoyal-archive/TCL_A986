
#ifndef _ALPHA_USER_H
#define _ALPHA_USER_H

#include <linux/sched.h>
#include <linux/ptrace.h>

#include <asm/page.h>
#include <asm/reg.h>

struct user {
	unsigned long	regs[EF_SIZE/8+32];	/* integer and fp regs */
	size_t		u_tsize;		/* text size (pages) */
	size_t		u_dsize;		/* data size (pages) */
	size_t		u_ssize;		/* stack size (pages) */
	unsigned long	start_code;		/* text starting address */
	unsigned long	start_data;		/* data starting address */
	unsigned long	start_stack;		/* stack starting address */
	long int	signal;			/* signal causing core dump */
	unsigned long	u_ar0;			/* help gdb find registers */
	unsigned long	magic;			/* identifies a core file */
	char		u_comm[32];		/* user command name */
};

#define NBPG			PAGE_SIZE
#define UPAGES			1
#define HOST_TEXT_START_ADDR	(u.start_code)
#define HOST_DATA_START_ADDR	(u.start_data)
#define HOST_STACK_END_ADDR	(u.start_stack + u.u_ssize * NBPG)

#endif /* _ALPHA_USER_H */
