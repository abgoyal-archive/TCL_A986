

#ifndef _ASM_SEGMENT_H
#define _ASM_SEGMENT_H


#ifndef __ASSEMBLY__

typedef struct {
	unsigned long seg;
} mm_segment_t;

#define MAKE_MM_SEG(s)	((mm_segment_t) { (s) })

#ifdef CONFIG_MMU
#define USER_DS			MAKE_MM_SEG(TASK_SIZE - 1)
#define KERNEL_DS		MAKE_MM_SEG(0xdfffffffUL)
#else
#define USER_DS			MAKE_MM_SEG(memory_end)
#define KERNEL_DS		MAKE_MM_SEG(0xe0000000UL)
#endif

#define get_ds()		(KERNEL_DS)
#define get_fs()		(__current_thread_info->addr_limit)
#define segment_eq(a,b)		((a).seg == (b).seg)
#define __kernel_ds_p()		segment_eq(get_fs(), KERNEL_DS)
#define get_addr_limit()	(get_fs().seg)

#define set_fs(_x)					\
do {							\
	__current_thread_info->addr_limit = (_x);	\
} while(0)


#endif /* __ASSEMBLY__ */
#endif /* _ASM_SEGMENT_H */
