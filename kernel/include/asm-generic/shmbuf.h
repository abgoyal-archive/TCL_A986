
#ifndef __ASM_GENERIC_SHMBUF_H
#define __ASM_GENERIC_SHMBUF_H

#include <asm/bitsperlong.h>


struct shmid64_ds {
	struct ipc64_perm	shm_perm;	/* operation perms */
	size_t			shm_segsz;	/* size of segment (bytes) */
	__kernel_time_t		shm_atime;	/* last attach time */
#if __BITS_PER_LONG != 64
	unsigned long		__unused1;
#endif
	__kernel_time_t		shm_dtime;	/* last detach time */
#if __BITS_PER_LONG != 64
	unsigned long		__unused2;
#endif
	__kernel_time_t		shm_ctime;	/* last change time */
#if __BITS_PER_LONG != 64
	unsigned long		__unused3;
#endif
	__kernel_pid_t		shm_cpid;	/* pid of creator */
	__kernel_pid_t		shm_lpid;	/* pid of last operator */
	unsigned long		shm_nattch;	/* no. of current attaches */
	unsigned long		__unused4;
	unsigned long		__unused5;
};

struct shminfo64 {
	unsigned long	shmmax;
	unsigned long	shmmin;
	unsigned long	shmmni;
	unsigned long	shmseg;
	unsigned long	shmall;
	unsigned long	__unused1;
	unsigned long	__unused2;
	unsigned long	__unused3;
	unsigned long	__unused4;
};

#endif /* __ASM_GENERIC_SHMBUF_H */
