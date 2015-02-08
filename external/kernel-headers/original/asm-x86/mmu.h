#ifndef _ASM_X86_MMU_H
#define _ASM_X86_MMU_H

#include <linux/spinlock.h>
#include <linux/mutex.h>

typedef struct { 
	void *ldt;
#ifdef CONFIG_X86_64
	rwlock_t ldtlock; 
#endif
	int size;
	struct mutex lock;
	void *vdso;
} mm_context_t;

#endif /* _ASM_X86_MMU_H */
