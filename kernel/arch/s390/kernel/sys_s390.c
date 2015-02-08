

#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/smp.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <linux/mman.h>
#include <linux/file.h>
#include <linux/utsname.h>
#include <linux/personality.h>
#include <linux/unistd.h>
#include <linux/ipc.h>
#include <trace/ipc.h>
#include <asm/uaccess.h>
#include "entry.h"

DEFINE_TRACE(ipc_call);


struct s390_mmap_arg_struct {
	unsigned long addr;
	unsigned long len;
	unsigned long prot;
	unsigned long flags;
	unsigned long fd;
	unsigned long offset;
};

SYSCALL_DEFINE1(mmap2, struct s390_mmap_arg_struct __user *, arg)
{
	struct s390_mmap_arg_struct a;
	int error = -EFAULT;

	if (copy_from_user(&a, arg, sizeof(a)))
		goto out;
	error = sys_mmap_pgoff(a.addr, a.len, a.prot, a.flags, a.fd, a.offset);
out:
	return error;
}

SYSCALL_DEFINE5(s390_ipc, uint, call, int, first, unsigned long, second,
		unsigned long, third, void __user *, ptr)
{
        struct ipc_kludge tmp;
	int ret;

        trace_ipc_call(call, first);

        switch (call) {
        case SEMOP:
		return sys_semtimedop(first, (struct sembuf __user *)ptr,
				       (unsigned)second, NULL);
	case SEMTIMEDOP:
		return sys_semtimedop(first, (struct sembuf __user *)ptr,
				       (unsigned)second,
				       (const struct timespec __user *) third);
        case SEMGET:
                return sys_semget(first, (int)second, third);
        case SEMCTL: {
                union semun fourth;
                if (!ptr)
                        return -EINVAL;
                if (get_user(fourth.__pad, (void __user * __user *) ptr))
                        return -EFAULT;
                return sys_semctl(first, (int)second, third, fourth);
        }
        case MSGSND:
		return sys_msgsnd (first, (struct msgbuf __user *) ptr,
                                   (size_t)second, third);
		break;
        case MSGRCV:
                if (!ptr)
                        return -EINVAL;
                if (copy_from_user (&tmp, (struct ipc_kludge __user *) ptr,
                                    sizeof (struct ipc_kludge)))
                        return -EFAULT;
                return sys_msgrcv (first, tmp.msgp,
                                   (size_t)second, tmp.msgtyp, third);
        case MSGGET:
                return sys_msgget((key_t)first, (int)second);
        case MSGCTL:
                return sys_msgctl(first, (int)second,
				   (struct msqid_ds __user *)ptr);

	case SHMAT: {
		ulong raddr;
		ret = do_shmat(first, (char __user *)ptr,
				(int)second, &raddr);
		if (ret)
			return ret;
		return put_user (raddr, (ulong __user *) third);
		break;
        }
	case SHMDT:
		return sys_shmdt ((char __user *)ptr);
	case SHMGET:
		return sys_shmget(first, (size_t)second, third);
	case SHMCTL:
		return sys_shmctl(first, (int)second,
                                   (struct shmid_ds __user *) ptr);
	default:
		return -ENOSYS;

	}

	return -EINVAL;
}

#ifdef CONFIG_64BIT
SYSCALL_DEFINE1(s390_personality, unsigned long, personality)
{
	int ret;

	if (current->personality == PER_LINUX32 && personality == PER_LINUX)
		personality = PER_LINUX32;
	ret = sys_personality(personality);
	if (ret == PER_LINUX32)
		ret = PER_LINUX;

	return ret;
}
#endif /* CONFIG_64BIT */

#ifndef CONFIG_64BIT

SYSCALL_DEFINE5(s390_fadvise64, int, fd, u32, offset_high, u32, offset_low,
		size_t, len, int, advice)
{
	return sys_fadvise64(fd, (u64) offset_high << 32 | offset_low,
			len, advice);
}

struct fadvise64_64_args {
	int fd;
	long long offset;
	long long len;
	int advice;
};

SYSCALL_DEFINE1(s390_fadvise64_64, struct fadvise64_64_args __user *, args)
{
	struct fadvise64_64_args a;

	if ( copy_from_user(&a, args, sizeof(a)) )
		return -EFAULT;
	return sys_fadvise64_64(a.fd, a.offset, a.len, a.advice);
}

SYSCALL_DEFINE(s390_fallocate)(int fd, int mode, loff_t offset,
			       u32 len_high, u32 len_low)
{
	return sys_fallocate(fd, mode, offset, ((u64)len_high << 32) | len_low);
}
#ifdef CONFIG_HAVE_SYSCALL_WRAPPERS
asmlinkage long SyS_s390_fallocate(long fd, long mode, loff_t offset,
				   long len_high, long len_low)
{
	return SYSC_s390_fallocate((int) fd, (int) mode, offset,
				   (u32) len_high, (u32) len_low);
}
SYSCALL_ALIAS(sys_s390_fallocate, SyS_s390_fallocate);
#endif

#endif
