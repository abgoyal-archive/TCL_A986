

#ifndef _LINUX_CAPABILITY_H
#define _LINUX_CAPABILITY_H

#include <linux/types.h>

struct task_struct;



#define _LINUX_CAPABILITY_VERSION_1  0x19980330
#define _LINUX_CAPABILITY_U32S_1     1

#define _LINUX_CAPABILITY_VERSION_2  0x20071026  /* deprecated - use v3 */
#define _LINUX_CAPABILITY_U32S_2     2

#define _LINUX_CAPABILITY_VERSION_3  0x20080522
#define _LINUX_CAPABILITY_U32S_3     2

typedef struct __user_cap_header_struct {
	__u32 version;
	int pid;
} __user *cap_user_header_t;

typedef struct __user_cap_data_struct {
        __u32 effective;
        __u32 permitted;
        __u32 inheritable;
} __user *cap_user_data_t;


#define XATTR_CAPS_SUFFIX "capability"
#define XATTR_NAME_CAPS XATTR_SECURITY_PREFIX XATTR_CAPS_SUFFIX

#define VFS_CAP_REVISION_MASK	0xFF000000
#define VFS_CAP_REVISION_SHIFT	24
#define VFS_CAP_FLAGS_MASK	~VFS_CAP_REVISION_MASK
#define VFS_CAP_FLAGS_EFFECTIVE	0x000001

#define VFS_CAP_REVISION_1	0x01000000
#define VFS_CAP_U32_1           1
#define XATTR_CAPS_SZ_1         (sizeof(__le32)*(1 + 2*VFS_CAP_U32_1))

#define VFS_CAP_REVISION_2	0x02000000
#define VFS_CAP_U32_2           2
#define XATTR_CAPS_SZ_2         (sizeof(__le32)*(1 + 2*VFS_CAP_U32_2))

#define XATTR_CAPS_SZ           XATTR_CAPS_SZ_2
#define VFS_CAP_U32             VFS_CAP_U32_2
#define VFS_CAP_REVISION	VFS_CAP_REVISION_2

struct vfs_cap_data {
	__le32 magic_etc;            /* Little endian */
	struct {
		__le32 permitted;    /* Little endian */
		__le32 inheritable;  /* Little endian */
	} data[VFS_CAP_U32];
};

#ifndef __KERNEL__

#define _LINUX_CAPABILITY_VERSION  _LINUX_CAPABILITY_VERSION_1
#define _LINUX_CAPABILITY_U32S     _LINUX_CAPABILITY_U32S_1

#else

#define _KERNEL_CAPABILITY_VERSION _LINUX_CAPABILITY_VERSION_3
#define _KERNEL_CAPABILITY_U32S    _LINUX_CAPABILITY_U32S_3

extern int file_caps_enabled;

typedef struct kernel_cap_struct {
	__u32 cap[_KERNEL_CAPABILITY_U32S];
} kernel_cap_t;

/* exact same as vfs_cap_data but in cpu endian and always filled completely */
struct cpu_vfs_cap_data {
	__u32 magic_etc;
	kernel_cap_t permitted;
	kernel_cap_t inheritable;
};

#define _USER_CAP_HEADER_SIZE  (sizeof(struct __user_cap_header_struct))
#define _KERNEL_CAP_T_SIZE     (sizeof(kernel_cap_t))

#endif




#define CAP_CHOWN            0


#define CAP_DAC_OVERRIDE     1


#define CAP_DAC_READ_SEARCH  2


#define CAP_FOWNER           3


#define CAP_FSETID           4


#define CAP_KILL             5

/* Allows setgid(2) manipulation */
/* Allows setgroups(2) */
/* Allows forged gids on socket credentials passing. */

#define CAP_SETGID           6

/* Allows set*uid(2) manipulation (including fsuid). */
/* Allows forged pids on socket credentials passing. */

#define CAP_SETUID           7




#define CAP_SETPCAP          8

/* Allow modification of S_IMMUTABLE and S_APPEND file attributes */

#define CAP_LINUX_IMMUTABLE  9

/* Allows binding to TCP/UDP sockets below 1024 */
/* Allows binding to ATM VCIs below 32 */

#define CAP_NET_BIND_SERVICE 10

/* Allow broadcasting, listen to multicast */

#define CAP_NET_BROADCAST    11

/* Allow interface configuration */
/* Allow administration of IP firewall, masquerading and accounting */
/* Allow setting debug option on sockets */
/* Allow modification of routing tables */
/* Allow binding to any address for transparent proxying */
/* Allow setting TOS (type of service) */
/* Allow setting promiscuous mode */
/* Allow clearing driver statistics */
/* Allow multicasting */
/* Allow read/write of device-specific registers */
/* Allow activation of ATM control sockets */

#define CAP_NET_ADMIN        12

/* Allow use of RAW sockets */
/* Allow use of PACKET sockets */

#define CAP_NET_RAW          13

/* Allow locking of shared memory segments */

#define CAP_IPC_LOCK         14

/* Override IPC ownership checks */

#define CAP_IPC_OWNER        15

/* Insert and remove kernel modules - modify kernel without limit */
#define CAP_SYS_MODULE       16

/* Allow ioperm/iopl access */
/* Allow sending USB messages to any device via /proc/bus/usb */

#define CAP_SYS_RAWIO        17

/* Allow use of chroot() */

#define CAP_SYS_CHROOT       18

/* Allow ptrace() of any process */

#define CAP_SYS_PTRACE       19

/* Allow configuration of process accounting */

#define CAP_SYS_PACCT        20

/* Allow configuration of the secure attention key */
/* Allow administration of the random device */
/* Allow examination and configuration of disk quotas */
/* Allow configuring the kernel's syslog (printk behaviour) */
/* Allow setting the domainname */
/* Allow setting the hostname */
/* Allow calling bdflush() */
/* Allow mount() and umount(), setting up new smb connection */
/* Allow some autofs root ioctls */
/* Allow nfsservctl */
/* Allow VM86_REQUEST_IRQ */
/* Allow to read/write pci config on alpha */
/* Allow irix_prctl on mips (setstacksize) */
/* Allow flushing all cache on m68k (sys_cacheflush) */
/* Allow removing semaphores */
/* Allow locking/unlocking of shared memory segment */
/* Allow turning swap on/off */
/* Allow forged pids on socket credentials passing */
/* Allow setting readahead and flushing buffers on block devices */
/* Allow setting geometry in floppy driver */
/* Allow turning DMA on/off in xd driver */
/* Allow tuning the ide driver */
/* Allow access to the nvram device */
/* Allow administration of apm_bios, serial and bttv (TV) device */
/* Allow manufacturer commands in isdn CAPI support driver */
/* Allow reading non-standardized portions of pci configuration space */
/* Allow DDI debug ioctl on sbpcd driver */
/* Allow setting up serial ports */
/* Allow sending raw qic-117 commands */
/* Allow setting encryption key on loopback filesystem */
/* Allow setting zone reclaim policy */

#define CAP_SYS_ADMIN        21

/* Allow use of reboot() */

#define CAP_SYS_BOOT         22

/* Allow setting cpu affinity on other processes */

#define CAP_SYS_NICE         23

/* Override resource limits. Set resource limits. */
/* Override quota limits. */
/* Override reserved space on ext2 filesystem */
/* Override size restrictions on IPC message queues */
/* Allow more than 64hz interrupts from the real-time clock */
/* Override max number of consoles on console allocation */
/* Override max number of keymaps */

#define CAP_SYS_RESOURCE     24

/* Allow manipulation of system clock */
/* Allow irix_stime on mips */
/* Allow setting the real-time clock */

#define CAP_SYS_TIME         25

/* Allow configuration of tty devices */
/* Allow vhangup() of tty */

#define CAP_SYS_TTY_CONFIG   26

/* Allow the privileged aspects of mknod() */

#define CAP_MKNOD            27

/* Allow taking of leases on files */

#define CAP_LEASE            28

#define CAP_AUDIT_WRITE      29

#define CAP_AUDIT_CONTROL    30

#define CAP_SETFCAP	     31


#define CAP_MAC_OVERRIDE     32


#define CAP_MAC_ADMIN        33

#define CAP_LAST_CAP         CAP_MAC_ADMIN

#define cap_valid(x) ((x) >= 0 && (x) <= CAP_LAST_CAP)


#define CAP_TO_INDEX(x)     ((x) >> 5)        /* 1 << 5 == bits in __u32 */
#define CAP_TO_MASK(x)      (1 << ((x) & 31)) /* mask for indexed __u32 */

#ifdef __KERNEL__


#define CAP_FOR_EACH_U32(__capi)  \
	for (__capi = 0; __capi < _KERNEL_CAPABILITY_U32S; ++__capi)


# define CAP_FS_MASK_B0     (CAP_TO_MASK(CAP_CHOWN)		\
			    | CAP_TO_MASK(CAP_MKNOD)		\
			    | CAP_TO_MASK(CAP_DAC_OVERRIDE)	\
			    | CAP_TO_MASK(CAP_DAC_READ_SEARCH)	\
			    | CAP_TO_MASK(CAP_FOWNER)		\
			    | CAP_TO_MASK(CAP_FSETID))

# define CAP_FS_MASK_B1     (CAP_TO_MASK(CAP_MAC_OVERRIDE))

#if _KERNEL_CAPABILITY_U32S != 2
# error Fix up hand-coded capability macro initializers
#else /* HAND-CODED capability initializers */

# define CAP_EMPTY_SET    ((kernel_cap_t){{ 0, 0 }})
# define CAP_FULL_SET     ((kernel_cap_t){{ ~0, ~0 }})
# define CAP_INIT_EFF_SET ((kernel_cap_t){{ ~CAP_TO_MASK(CAP_SETPCAP), ~0 }})
# define CAP_FS_SET       ((kernel_cap_t){{ CAP_FS_MASK_B0 \
				    | CAP_TO_MASK(CAP_LINUX_IMMUTABLE), \
				    CAP_FS_MASK_B1 } })
# define CAP_NFSD_SET     ((kernel_cap_t){{ CAP_FS_MASK_B0 \
				    | CAP_TO_MASK(CAP_SYS_RESOURCE), \
				    CAP_FS_MASK_B1 } })

#endif /* _KERNEL_CAPABILITY_U32S != 2 */

#define CAP_INIT_INH_SET    CAP_EMPTY_SET

# define cap_clear(c)         do { (c) = __cap_empty_set; } while (0)
# define cap_set_full(c)      do { (c) = __cap_full_set; } while (0)
# define cap_set_init_eff(c)  do { (c) = __cap_init_eff_set; } while (0)

#define cap_raise(c, flag)  ((c).cap[CAP_TO_INDEX(flag)] |= CAP_TO_MASK(flag))
#define cap_lower(c, flag)  ((c).cap[CAP_TO_INDEX(flag)] &= ~CAP_TO_MASK(flag))
#define cap_raised(c, flag) ((c).cap[CAP_TO_INDEX(flag)] & CAP_TO_MASK(flag))

#define CAP_BOP_ALL(c, a, b, OP)                                    \
do {                                                                \
	unsigned __capi;                                            \
	CAP_FOR_EACH_U32(__capi) {                                  \
		c.cap[__capi] = a.cap[__capi] OP b.cap[__capi];     \
	}                                                           \
} while (0)

#define CAP_UOP_ALL(c, a, OP)                                       \
do {                                                                \
	unsigned __capi;                                            \
	CAP_FOR_EACH_U32(__capi) {                                  \
		c.cap[__capi] = OP a.cap[__capi];                   \
	}                                                           \
} while (0)

static inline kernel_cap_t cap_combine(const kernel_cap_t a,
				       const kernel_cap_t b)
{
	kernel_cap_t dest;
	CAP_BOP_ALL(dest, a, b, |);
	return dest;
}

static inline kernel_cap_t cap_intersect(const kernel_cap_t a,
					 const kernel_cap_t b)
{
	kernel_cap_t dest;
	CAP_BOP_ALL(dest, a, b, &);
	return dest;
}

static inline kernel_cap_t cap_drop(const kernel_cap_t a,
				    const kernel_cap_t drop)
{
	kernel_cap_t dest;
	CAP_BOP_ALL(dest, a, drop, &~);
	return dest;
}

static inline kernel_cap_t cap_invert(const kernel_cap_t c)
{
	kernel_cap_t dest;
	CAP_UOP_ALL(dest, c, ~);
	return dest;
}

static inline int cap_isclear(const kernel_cap_t a)
{
	unsigned __capi;
	CAP_FOR_EACH_U32(__capi) {
		if (a.cap[__capi] != 0)
			return 0;
	}
	return 1;
}

static inline int cap_issubset(const kernel_cap_t a, const kernel_cap_t set)
{
	kernel_cap_t dest;
	dest = cap_drop(a, set);
	return cap_isclear(dest);
}

/* Used to decide between falling back on the old suser() or fsuser(). */

static inline int cap_is_fs_cap(int cap)
{
	const kernel_cap_t __cap_fs_set = CAP_FS_SET;
	return !!(CAP_TO_MASK(cap) & __cap_fs_set.cap[CAP_TO_INDEX(cap)]);
}

static inline kernel_cap_t cap_drop_fs_set(const kernel_cap_t a)
{
	const kernel_cap_t __cap_fs_set = CAP_FS_SET;
	return cap_drop(a, __cap_fs_set);
}

static inline kernel_cap_t cap_raise_fs_set(const kernel_cap_t a,
					    const kernel_cap_t permitted)
{
	const kernel_cap_t __cap_fs_set = CAP_FS_SET;
	return cap_combine(a,
			   cap_intersect(permitted, __cap_fs_set));
}

static inline kernel_cap_t cap_drop_nfsd_set(const kernel_cap_t a)
{
	const kernel_cap_t __cap_fs_set = CAP_NFSD_SET;
	return cap_drop(a, __cap_fs_set);
}

static inline kernel_cap_t cap_raise_nfsd_set(const kernel_cap_t a,
					      const kernel_cap_t permitted)
{
	const kernel_cap_t __cap_nfsd_set = CAP_NFSD_SET;
	return cap_combine(a,
			   cap_intersect(permitted, __cap_nfsd_set));
}

extern const kernel_cap_t __cap_empty_set;
extern const kernel_cap_t __cap_full_set;
extern const kernel_cap_t __cap_init_eff_set;

#define has_capability(t, cap) (security_real_capable((t), (cap)) == 0)

#define has_capability_noaudit(t, cap) \
	(security_real_capable_noaudit((t), (cap)) == 0)

extern int capable(int cap);

/* audit system wants to get cap info from files as well */
struct dentry;
extern int get_vfs_caps_from_disk(const struct dentry *dentry, struct cpu_vfs_cap_data *cpu_caps);

#endif /* __KERNEL__ */

#endif /* !_LINUX_CAPABILITY_H */
