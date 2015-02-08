
#ifndef _LINUX_CAPABILITY_H
#define _LINUX_CAPABILITY_H

#include <linux/types.h>
#include <linux/compiler.h>


 
#define _LINUX_CAPABILITY_VERSION  0x19980330

typedef struct __user_cap_header_struct {
	__u32 version;
	int pid;
} __user *cap_user_header_t;
 
typedef struct __user_cap_data_struct {
        __u32 effective;
        __u32 permitted;
        __u32 inheritable;
} __user *cap_user_data_t;
  
#ifdef __KERNEL__

#include <linux/spinlock.h>
#include <asm/current.h>

/* #define STRICT_CAP_T_TYPECHECKS */

#ifdef STRICT_CAP_T_TYPECHECKS

typedef struct kernel_cap_struct {
	__u32 cap;
} kernel_cap_t;

#else

typedef __u32 kernel_cap_t;

#endif
  
#define _USER_CAP_HEADER_SIZE  (2*sizeof(__u32))
#define _KERNEL_CAP_T_SIZE     (sizeof(kernel_cap_t))

#endif




#define CAP_CHOWN            0


#define CAP_DAC_OVERRIDE     1


#define CAP_DAC_READ_SEARCH  2
    

#define CAP_FOWNER           3


#define CAP_FSETID           4

/* Used to decide between falling back on the old suser() or fsuser(). */

#define CAP_FS_MASK          0x1f


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
/* Modify cap_bset */
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

#ifdef __KERNEL__
extern kernel_cap_t cap_bset;

 
#ifdef STRICT_CAP_T_TYPECHECKS

#define to_cap_t(x) { x }
#define cap_t(x) (x).cap

#else

#define to_cap_t(x) (x)
#define cap_t(x) (x)

#endif

#define CAP_EMPTY_SET       to_cap_t(0)
#define CAP_FULL_SET        to_cap_t(~0)
#define CAP_INIT_EFF_SET    to_cap_t(~0 & ~CAP_TO_MASK(CAP_SETPCAP))
#define CAP_INIT_INH_SET    to_cap_t(0)

#define CAP_TO_MASK(x) (1 << (x))
#define cap_raise(c, flag)   (cap_t(c) |=  CAP_TO_MASK(flag))
#define cap_lower(c, flag)   (cap_t(c) &= ~CAP_TO_MASK(flag))
#define cap_raised(c, flag)  (cap_t(c) & CAP_TO_MASK(flag))

static inline kernel_cap_t cap_combine(kernel_cap_t a, kernel_cap_t b)
{
     kernel_cap_t dest;
     cap_t(dest) = cap_t(a) | cap_t(b);
     return dest;
}

static inline kernel_cap_t cap_intersect(kernel_cap_t a, kernel_cap_t b)
{
     kernel_cap_t dest;
     cap_t(dest) = cap_t(a) & cap_t(b);
     return dest;
}

static inline kernel_cap_t cap_drop(kernel_cap_t a, kernel_cap_t drop)
{
     kernel_cap_t dest;
     cap_t(dest) = cap_t(a) & ~cap_t(drop);
     return dest;
}

static inline kernel_cap_t cap_invert(kernel_cap_t c)
{
     kernel_cap_t dest;
     cap_t(dest) = ~cap_t(c);
     return dest;
}

#define cap_isclear(c)       (!cap_t(c))
#define cap_issubset(a,set)  (!(cap_t(a) & ~cap_t(set)))

#define cap_clear(c)         do { cap_t(c) =  0; } while(0)
#define cap_set_full(c)      do { cap_t(c) = ~0; } while(0)
#define cap_mask(c,mask)     do { cap_t(c) &= cap_t(mask); } while(0)

#define cap_is_fs_cap(c)     (CAP_TO_MASK(c) & CAP_FS_MASK)

int capable(int cap);
int __capable(struct task_struct *t, int cap);

#endif /* __KERNEL__ */

#endif /* !_LINUX_CAPABILITY_H */
