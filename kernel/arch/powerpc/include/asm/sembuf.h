
#ifndef _ASM_POWERPC_SEMBUF_H
#define _ASM_POWERPC_SEMBUF_H



struct semid64_ds {
	struct ipc64_perm sem_perm;	/* permissions .. see ipc.h */
#ifndef __powerpc64__
	unsigned long	__unused1;
#endif
	__kernel_time_t	sem_otime;	/* last semop time */
#ifndef __powerpc64__
	unsigned long	__unused2;
#endif
	__kernel_time_t	sem_ctime;	/* last change time */
	unsigned long	sem_nsems;	/* no. of semaphores in array */
	unsigned long	__unused3;
	unsigned long	__unused4;
};

#endif	/* _ASM_POWERPC_SEMBUF_H */
