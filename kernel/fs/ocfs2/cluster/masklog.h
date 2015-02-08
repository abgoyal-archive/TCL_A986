

#ifndef O2CLUSTER_MASKLOG_H
#define O2CLUSTER_MASKLOG_H


/* for task_struct */
#include <linux/sched.h>

/* bits that are frequently given and infrequently matched in the low word */
/* NOTE: If you add a flag, you need to also update mlog.c! */
#define ML_ENTRY	0x0000000000000001ULL /* func call entry */
#define ML_EXIT		0x0000000000000002ULL /* func call exit */
#define ML_TCP		0x0000000000000004ULL /* net cluster/tcp.c */
#define ML_MSG		0x0000000000000008ULL /* net network messages */
#define ML_SOCKET	0x0000000000000010ULL /* net socket lifetime */
#define ML_HEARTBEAT	0x0000000000000020ULL /* hb all heartbeat tracking */
#define ML_HB_BIO	0x0000000000000040ULL /* hb io tracing */
#define ML_DLMFS	0x0000000000000080ULL /* dlm user dlmfs */
#define ML_DLM		0x0000000000000100ULL /* dlm general debugging */
#define ML_DLM_DOMAIN	0x0000000000000200ULL /* dlm domain debugging */
#define ML_DLM_THREAD	0x0000000000000400ULL /* dlm domain thread */
#define ML_DLM_MASTER	0x0000000000000800ULL /* dlm master functions */
#define ML_DLM_RECOVERY	0x0000000000001000ULL /* dlm master functions */
#define ML_AIO		0x0000000000002000ULL /* ocfs2 aio read and write */
#define ML_JOURNAL	0x0000000000004000ULL /* ocfs2 journalling functions */
#define ML_DISK_ALLOC	0x0000000000008000ULL /* ocfs2 disk allocation */
#define ML_SUPER	0x0000000000010000ULL /* ocfs2 mount / umount */
#define ML_FILE_IO	0x0000000000020000ULL /* ocfs2 file I/O */
#define ML_EXTENT_MAP	0x0000000000040000ULL /* ocfs2 extent map caching */
#define ML_DLM_GLUE	0x0000000000080000ULL /* ocfs2 dlm glue layer */
#define ML_BH_IO	0x0000000000100000ULL /* ocfs2 buffer I/O */
#define ML_UPTODATE	0x0000000000200000ULL /* ocfs2 caching sequence #'s */
#define ML_NAMEI	0x0000000000400000ULL /* ocfs2 directory / namespace */
#define ML_INODE	0x0000000000800000ULL /* ocfs2 inode manipulation */
#define ML_VOTE		0x0000000001000000ULL /* ocfs2 node messaging  */
#define ML_DCACHE	0x0000000002000000ULL /* ocfs2 dcache operations */
#define ML_CONN		0x0000000004000000ULL /* net connection management */
#define ML_QUORUM	0x0000000008000000ULL /* net connection quorum */
#define ML_EXPORT	0x0000000010000000ULL /* ocfs2 export operations */
#define ML_XATTR	0x0000000020000000ULL /* ocfs2 extended attributes */
#define ML_QUOTA	0x0000000040000000ULL /* ocfs2 quota operations */
#define ML_REFCOUNT	0x0000000080000000ULL /* refcount tree operations */
#define ML_BASTS	0x0000001000000000ULL /* dlmglue asts and basts */
/* bits that are infrequently given and frequently matched in the high word */
#define ML_ERROR	0x0000000100000000ULL /* sent to KERN_ERR */
#define ML_NOTICE	0x0000000200000000ULL /* setn to KERN_NOTICE */
#define ML_KTHREAD	0x0000000400000000ULL /* kernel thread activity */
#define	ML_RESERVATIONS	0x0000000800000000ULL /* ocfs2 alloc reservations */

#define MLOG_INITIAL_AND_MASK (ML_ERROR|ML_NOTICE)
#define MLOG_INITIAL_NOT_MASK (ML_ENTRY|ML_EXIT)
#ifndef MLOG_MASK_PREFIX
#define MLOG_MASK_PREFIX 0
#endif

#if defined(CONFIG_OCFS2_DEBUG_MASKLOG)
#define ML_ALLOWED_BITS ~0
#else
#define ML_ALLOWED_BITS (ML_ERROR|ML_NOTICE)
#endif

#define MLOG_MAX_BITS 64

struct mlog_bits {
	unsigned long words[MLOG_MAX_BITS / BITS_PER_LONG];
};

extern struct mlog_bits mlog_and_bits, mlog_not_bits;

#if BITS_PER_LONG == 32

#define __mlog_test_u64(mask, bits)			\
	( (u32)(mask & 0xffffffff) & bits.words[0] || 	\
	  ((u64)(mask) >> 32) & bits.words[1] )
#define __mlog_set_u64(mask, bits) do {			\
	bits.words[0] |= (u32)(mask & 0xffffffff);	\
       	bits.words[1] |= (u64)(mask) >> 32;		\
} while (0)
#define __mlog_clear_u64(mask, bits) do {		\
	bits.words[0] &= ~((u32)(mask & 0xffffffff));	\
       	bits.words[1] &= ~((u64)(mask) >> 32);		\
} while (0)
#define MLOG_BITS_RHS(mask) {				\
	{						\
		[0] = (u32)(mask & 0xffffffff),		\
		[1] = (u64)(mask) >> 32,		\
	}						\
}

#else /* 32bit long above, 64bit long below */

#define __mlog_test_u64(mask, bits)	((mask) & bits.words[0])
#define __mlog_set_u64(mask, bits) do {		\
	bits.words[0] |= (mask);		\
} while (0)
#define __mlog_clear_u64(mask, bits) do {	\
	bits.words[0] &= ~(mask);		\
} while (0)
#define MLOG_BITS_RHS(mask) { { (mask) } }

#endif

#define __mlog_cpu_guess ({		\
	unsigned long _cpu = get_cpu();	\
	put_cpu();			\
	_cpu;				\
})

#define __mlog_printk(level, fmt, args...)				\
	printk(level "(%s,%u,%lu):%s:%d " fmt, current->comm,		\
	       task_pid_nr(current), __mlog_cpu_guess,			\
	       __PRETTY_FUNCTION__, __LINE__ , ##args)

#define mlog(mask, fmt, args...) do {					\
	u64 __m = MLOG_MASK_PREFIX | (mask);				\
	if ((__m & ML_ALLOWED_BITS) &&					\
	    __mlog_test_u64(__m, mlog_and_bits) &&			\
	    !__mlog_test_u64(__m, mlog_not_bits)) {			\
		if (__m & ML_ERROR)					\
			__mlog_printk(KERN_ERR, "ERROR: "fmt , ##args);	\
		else if (__m & ML_NOTICE)				\
			__mlog_printk(KERN_NOTICE, fmt , ##args);	\
		else __mlog_printk(KERN_INFO, fmt , ##args);		\
	}								\
} while (0)

#define mlog_errno(st) do {						\
	int _st = (st);							\
	if (_st != -ERESTARTSYS && _st != -EINTR &&			\
	    _st != AOP_TRUNCATED_PAGE && _st != -ENOSPC)		\
		mlog(ML_ERROR, "status = %lld\n", (long long)_st);	\
} while (0)

#if defined(CONFIG_OCFS2_DEBUG_MASKLOG)
#define mlog_entry(fmt, args...) do {					\
	mlog(ML_ENTRY, "ENTRY:" fmt , ##args);				\
} while (0)

#define mlog_entry_void() do {						\
	mlog(ML_ENTRY, "ENTRY:\n");					\
} while (0)

#if !defined(__CHECKER__)
#define mlog_exit(st) do {						     \
	if (__builtin_types_compatible_p(typeof(st), unsigned long))	     \
		mlog(ML_EXIT, "EXIT: %lu\n", (unsigned long) (st));	     \
	else if (__builtin_types_compatible_p(typeof(st), signed long))      \
		mlog(ML_EXIT, "EXIT: %ld\n", (signed long) (st));	     \
	else if (__builtin_types_compatible_p(typeof(st), unsigned int)	     \
		 || __builtin_types_compatible_p(typeof(st), unsigned short) \
		 || __builtin_types_compatible_p(typeof(st), unsigned char)) \
		mlog(ML_EXIT, "EXIT: %u\n", (unsigned int) (st));	     \
	else if (__builtin_types_compatible_p(typeof(st), signed int)	     \
		 || __builtin_types_compatible_p(typeof(st), signed short)   \
		 || __builtin_types_compatible_p(typeof(st), signed char))   \
		mlog(ML_EXIT, "EXIT: %d\n", (signed int) (st));		     \
	else if (__builtin_types_compatible_p(typeof(st), long long))	     \
		mlog(ML_EXIT, "EXIT: %lld\n", (long long) (st));	     \
	else								     \
		mlog(ML_EXIT, "EXIT: %llu\n", (unsigned long long) (st));    \
} while (0)
#else
#define mlog_exit(st) do {						     \
	mlog(ML_EXIT, "EXIT: %lld\n", (long long) (st));		     \
} while (0)
#endif

#define mlog_exit_ptr(ptr) do {						\
	mlog(ML_EXIT, "EXIT: %p\n", ptr);				\
} while (0)

#define mlog_exit_void() do {						\
	mlog(ML_EXIT, "EXIT\n");					\
} while (0)
#else
#define mlog_entry(...)  do { } while (0)
#define mlog_entry_void(...)  do { } while (0)
#define mlog_exit(...)  do { } while (0)
#define mlog_exit_ptr(...)  do { } while (0)
#define mlog_exit_void(...)  do { } while (0)
#endif  /* defined(CONFIG_OCFS2_DEBUG_MASKLOG) */

#define mlog_bug_on_msg(cond, fmt, args...) do {			\
	if (cond) {							\
		mlog(ML_ERROR, "bug expression: " #cond "\n");		\
		mlog(ML_ERROR, fmt, ##args);				\
		BUG();							\
	}								\
} while (0)

#include <linux/kobject.h>
#include <linux/sysfs.h>
int mlog_sys_init(struct kset *o2cb_subsys);
void mlog_sys_shutdown(void);

#endif /* O2CLUSTER_MASKLOG_H */
