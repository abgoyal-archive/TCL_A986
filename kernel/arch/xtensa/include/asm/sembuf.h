

#ifndef _XTENSA_SEMBUF_H
#define _XTENSA_SEMBUF_H

#include <asm/byteorder.h>

struct semid64_ds {
	struct ipc64_perm sem_perm;		/* permissions .. see ipc.h */
#ifdef __XTENSA_EL__
	__kernel_time_t	sem_otime;		/* last semop time */
	unsigned long	__unused1;
	__kernel_time_t	sem_ctime;		/* last change time */
	unsigned long	__unused2;
#else
	unsigned long	__unused1;
	__kernel_time_t	sem_otime;		/* last semop time */
	unsigned long	__unused2;
	__kernel_time_t	sem_ctime;		/* last change time */
#endif
	unsigned long	sem_nsems;		/* no. of semaphores in array */
	unsigned long	__unused3;
	unsigned long	__unused4;
};

#endif /* __ASM_XTENSA_SEMBUF_H */
