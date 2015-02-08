
#ifndef _H8300_SEMBUF_H
#define _H8300_SEMBUF_H


struct semid64_ds {
	struct ipc64_perm sem_perm;		/* permissions .. see ipc.h */
	__kernel_time_t	sem_otime;		/* last semop time */
	unsigned long	__unused1;
	__kernel_time_t	sem_ctime;		/* last change time */
	unsigned long	__unused2;
	unsigned long	sem_nsems;		/* no. of semaphores in array */
	unsigned long	__unused3;
	unsigned long	__unused4;
};

#endif /* _H8300_SEMBUF_H */
