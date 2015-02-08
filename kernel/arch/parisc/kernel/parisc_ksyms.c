

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/syscalls.h>

#include <linux/string.h>
EXPORT_SYMBOL(memset);

#include <asm/atomic.h>
EXPORT_SYMBOL(__xchg8);
EXPORT_SYMBOL(__xchg32);
EXPORT_SYMBOL(__cmpxchg_u32);
#ifdef CONFIG_SMP
EXPORT_SYMBOL(__atomic_hash);
#endif
#ifdef CONFIG_64BIT
EXPORT_SYMBOL(__xchg64);
EXPORT_SYMBOL(__cmpxchg_u64);
#endif

#include <asm/uaccess.h>
EXPORT_SYMBOL(lstrncpy_from_user);
EXPORT_SYMBOL(lclear_user);
EXPORT_SYMBOL(lstrnlen_user);

/* Global fixups */
extern void fixup_get_user_skip_1(void);
extern void fixup_get_user_skip_2(void);
extern void fixup_put_user_skip_1(void);
extern void fixup_put_user_skip_2(void);
EXPORT_SYMBOL(fixup_get_user_skip_1);
EXPORT_SYMBOL(fixup_get_user_skip_2);
EXPORT_SYMBOL(fixup_put_user_skip_1);
EXPORT_SYMBOL(fixup_put_user_skip_2);

#ifndef CONFIG_64BIT
/* Needed so insmod can set dp value */
extern int $global$;
EXPORT_SYMBOL($global$);
#endif

#include <asm/io.h>
EXPORT_SYMBOL(memcpy_toio);
EXPORT_SYMBOL(memcpy_fromio);
EXPORT_SYMBOL(memset_io);

extern void $$divI(void);
extern void $$divU(void);
extern void $$remI(void);
extern void $$remU(void);
extern void $$mulI(void);
extern void $$divU_3(void);
extern void $$divU_5(void);
extern void $$divU_6(void);
extern void $$divU_9(void);
extern void $$divU_10(void);
extern void $$divU_12(void);
extern void $$divU_7(void);
extern void $$divU_14(void);
extern void $$divU_15(void);
extern void $$divI_3(void);
extern void $$divI_5(void);
extern void $$divI_6(void);
extern void $$divI_7(void);
extern void $$divI_9(void);
extern void $$divI_10(void);
extern void $$divI_12(void);
extern void $$divI_14(void);
extern void $$divI_15(void);

EXPORT_SYMBOL($$divI);
EXPORT_SYMBOL($$divU);
EXPORT_SYMBOL($$remI);
EXPORT_SYMBOL($$remU);
EXPORT_SYMBOL($$mulI);
EXPORT_SYMBOL($$divU_3);
EXPORT_SYMBOL($$divU_5);
EXPORT_SYMBOL($$divU_6);
EXPORT_SYMBOL($$divU_9);
EXPORT_SYMBOL($$divU_10);
EXPORT_SYMBOL($$divU_12);
EXPORT_SYMBOL($$divU_7);
EXPORT_SYMBOL($$divU_14);
EXPORT_SYMBOL($$divU_15);
EXPORT_SYMBOL($$divI_3);
EXPORT_SYMBOL($$divI_5);
EXPORT_SYMBOL($$divI_6);
EXPORT_SYMBOL($$divI_7);
EXPORT_SYMBOL($$divI_9);
EXPORT_SYMBOL($$divI_10);
EXPORT_SYMBOL($$divI_12);
EXPORT_SYMBOL($$divI_14);
EXPORT_SYMBOL($$divI_15);

extern void __ashrdi3(void);
extern void __ashldi3(void);
extern void __lshrdi3(void);
extern void __muldi3(void);

EXPORT_SYMBOL(__ashrdi3);
EXPORT_SYMBOL(__ashldi3);
EXPORT_SYMBOL(__lshrdi3);
EXPORT_SYMBOL(__muldi3);

asmlinkage void * __canonicalize_funcptr_for_compare(void *);
EXPORT_SYMBOL(__canonicalize_funcptr_for_compare);

#ifdef CONFIG_64BIT
extern void __divdi3(void);
extern void __udivdi3(void);
extern void __umoddi3(void);
extern void __moddi3(void);

EXPORT_SYMBOL(__divdi3);
EXPORT_SYMBOL(__udivdi3);
EXPORT_SYMBOL(__umoddi3);
EXPORT_SYMBOL(__moddi3);
#endif

#ifndef CONFIG_64BIT
extern void $$dyncall(void);
EXPORT_SYMBOL($$dyncall);
#endif

#ifdef CONFIG_DISCONTIGMEM
#include <asm/mmzone.h>
EXPORT_SYMBOL(node_data);
EXPORT_SYMBOL(pfnnid_map);
#endif

#ifdef CONFIG_FUNCTION_TRACER
extern void _mcount(void);
EXPORT_SYMBOL(_mcount);
#endif

/* from pacache.S -- needed for copy_page */
EXPORT_SYMBOL(copy_user_page_asm);
