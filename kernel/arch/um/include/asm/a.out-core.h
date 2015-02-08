

#ifndef __UM_A_OUT_CORE_H
#define __UM_A_OUT_CORE_H

#ifdef __KERNEL__

#include <linux/user.h>

static inline void aout_dump_thread(struct pt_regs *regs, struct user *u)
{
}

#endif /* __KERNEL__ */
#endif /* __UM_A_OUT_CORE_H */
