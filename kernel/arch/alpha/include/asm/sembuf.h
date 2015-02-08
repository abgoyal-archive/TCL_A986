
#ifndef _ALPHA_SEMBUF_H
#define _ALPHA_SEMBUF_H


struct semid64_ds {
	struct ipc64_perm sem_perm;		/* permissions .. see ipc.h */
	__kernel_time_t	sem_otime;		/* last semop time */
	__kernel_time_t	sem_ctime;		/* last change time */
	unsigned long	sem_nsems;		/* no. of semaphores in array */
	unsigned long	__unused1;
	unsigned long	__unused2;
};

#endif /* _ALPHA_SEMBUF_H */
