
#ifndef _ASM_BUG_H
#define _ASM_BUG_H

#include <linux/linkage.h>

#ifdef CONFIG_BUG
extern asmlinkage void __debug_bug_trap(int signr);

#ifdef CONFIG_NO_KERNEL_MSG
#define	_debug_bug_printk()
#else
extern void __debug_bug_printk(const char *file, unsigned line);
#define	_debug_bug_printk() __debug_bug_printk(__FILE__, __LINE__)
#endif

#define _debug_bug_trap(signr)			\
do {						\
	__debug_bug_trap(signr);		\
	asm volatile("nop");			\
} while(1)

#define HAVE_ARCH_BUG
#define BUG()					\
do {						\
	_debug_bug_printk();			\
	_debug_bug_trap(6 /*SIGABRT*/);		\
} while (0)

#ifdef CONFIG_GDBSTUB
#define HAVE_ARCH_KGDB_RAISE
#define kgdb_raise(signr) do { _debug_bug_trap(signr); } while(0)

#define HAVE_ARCH_KGDB_BAD_PAGE
#define kgdb_bad_page(page) do { kgdb_raise(SIGABRT); } while(0)
#endif

#endif /* CONFIG_BUG */

#include <asm-generic/bug.h>

#endif
