
#ifndef _FS_CEPH_DEBUG_H
#define _FS_CEPH_DEBUG_H

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#ifdef CONFIG_CEPH_FS_PRETTYDEBUG


# if defined(DEBUG) || defined(CONFIG_DYNAMIC_DEBUG)
extern const char *ceph_file_part(const char *s, int len);
#  define dout(fmt, ...)						\
	pr_debug(" %12.12s:%-4d : " fmt,				\
		 ceph_file_part(__FILE__, sizeof(__FILE__)),		\
		 __LINE__, ##__VA_ARGS__)
# else
/* faux printk call just to see any compiler warnings. */
#  define dout(fmt, ...)	do {				\
		if (0)						\
			printk(KERN_DEBUG fmt, ##__VA_ARGS__);	\
	} while (0)
# endif

#else

# define dout(fmt, ...)	pr_debug(" " fmt, ##__VA_ARGS__)

#endif

#endif
