

/* This makes a dentry parent/child name pair. Useful for debugging printk's */
#define DENTRY_PATH(dentry) \
	(dentry)->d_parent->d_name.name,(dentry)->d_name.name

#ifdef SMBFS_PARANOIA
# define PARANOIA(f, a...) printk(KERN_NOTICE "%s: " f, __func__ , ## a)
#else
# define PARANOIA(f, a...) do { ; } while(0)
#endif

/* lots of debug messages */
#ifdef SMBFS_DEBUG_VERBOSE
# define VERBOSE(f, a...) printk(KERN_DEBUG "%s: " f, __func__ , ## a)
#else
# define VERBOSE(f, a...) do { ; } while(0)
#endif

#ifdef SMBFS_DEBUG
#define DEBUG1(f, a...) printk(KERN_DEBUG "%s: " f, __func__ , ## a)
#else
#define DEBUG1(f, a...) do { ; } while(0)
#endif
