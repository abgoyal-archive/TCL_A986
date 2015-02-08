
#ifndef __ASM_GENERIC_SEMBUF_H
#define __ASM_GENERIC_SEMBUF_H

#include <asm/bitsperlong.h>

struct semid64_ds {
	struct ipc64_perm sem_perm;	/* permissions .. see ipc.h */
	__kernel_time_t	sem_otime;	/* last semop time */
#if __BITS_PER_LONG != 64
	unsigned long	__unused1;
#endif
	__kernel_time_t	sem_ctime;	/* last change time */
#if __BITS_PER_LONG != 64
	unsigned long	__unused2;
#endif
	unsigned long	sem_nsems;	/* no. of semaphores in array */
	unsigned long	__unused3;
	unsigned long	__unused4;
};

#endif /* __ASM_GENERIC_SEMBUF_H */
