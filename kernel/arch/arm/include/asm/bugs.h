
#ifndef __ASM_BUGS_H
#define __ASM_BUGS_H

#ifdef CONFIG_MMU
extern void check_writebuffer_bugs(void);

#define check_bugs() check_writebuffer_bugs()
#else
#define check_bugs() do { } while (0)
#endif

#endif
