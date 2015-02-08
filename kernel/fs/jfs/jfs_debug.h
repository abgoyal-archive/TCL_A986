
#ifndef _H_JFS_DEBUG
#define _H_JFS_DEBUG


#if defined(CONFIG_PROC_FS) && (defined(CONFIG_JFS_DEBUG) || defined(CONFIG_JFS_STATISTICS))
#define PROC_FS_JFS
extern void jfs_proc_init(void);
extern void jfs_proc_clean(void);
#endif

#define assert(p) do {	\
	if (!(p)) {	\
		printk(KERN_CRIT "BUG at %s:%d assert(%s)\n",	\
		       __FILE__, __LINE__, #p);			\
		BUG();	\
	}		\
} while (0)

#ifdef CONFIG_JFS_DEBUG
#define ASSERT(p) assert(p)

/* printk verbosity */
#define JFS_LOGLEVEL_ERR 1
#define JFS_LOGLEVEL_WARN 2
#define JFS_LOGLEVEL_DEBUG 3
#define JFS_LOGLEVEL_INFO 4

extern int jfsloglevel;

extern const struct file_operations jfs_txanchor_proc_fops;

/* information message: e.g., configuration, major event */
#define jfs_info(fmt, arg...) do {			\
	if (jfsloglevel >= JFS_LOGLEVEL_INFO)		\
		printk(KERN_INFO fmt "\n", ## arg);	\
} while (0)

/* debug message: ad hoc */
#define jfs_debug(fmt, arg...) do {			\
	if (jfsloglevel >= JFS_LOGLEVEL_DEBUG)		\
		printk(KERN_DEBUG fmt "\n", ## arg);	\
} while (0)

/* warn message: */
#define jfs_warn(fmt, arg...) do {			\
	if (jfsloglevel >= JFS_LOGLEVEL_WARN)		\
		printk(KERN_WARNING fmt "\n", ## arg);	\
} while (0)

/* error event message: e.g., i/o error */
#define jfs_err(fmt, arg...) do {			\
	if (jfsloglevel >= JFS_LOGLEVEL_ERR)		\
		printk(KERN_ERR fmt "\n", ## arg);	\
} while (0)

#else				/* CONFIG_JFS_DEBUG */
#define ASSERT(p) do {} while (0)
#define jfs_info(fmt, arg...) do {} while (0)
#define jfs_debug(fmt, arg...) do {} while (0)
#define jfs_warn(fmt, arg...) do {} while (0)
#define jfs_err(fmt, arg...) do {} while (0)
#endif				/* CONFIG_JFS_DEBUG */

#ifdef	CONFIG_JFS_STATISTICS
extern const struct file_operations jfs_lmstats_proc_fops;
extern const struct file_operations jfs_txstats_proc_fops;
extern const struct file_operations jfs_mpstat_proc_fops;
extern const struct file_operations jfs_xtstat_proc_fops;

#define	INCREMENT(x)		((x)++)
#define	DECREMENT(x)		((x)--)
#define	HIGHWATERMARK(x,y)	((x) = max((x), (y)))
#else
#define	INCREMENT(x)
#define	DECREMENT(x)
#define	HIGHWATERMARK(x,y)
#endif				/* CONFIG_JFS_STATISTICS */

#endif				/* _H_JFS_DEBUG */
