
#ifndef _S390_SEMBUF_H
#define _S390_SEMBUF_H


struct semid64_ds {
	struct ipc64_perm sem_perm;		/* permissions .. see ipc.h */
	__kernel_time_t	sem_otime;		/* last semop time */
#ifndef __s390x__
	unsigned long	__unused1;
#endif /* ! __s390x__ */
	__kernel_time_t	sem_ctime;		/* last change time */
#ifndef __s390x__
	unsigned long	__unused2;
#endif /* ! __s390x__ */
	unsigned long	sem_nsems;		/* no. of semaphores in array */
	unsigned long	__unused3;
	unsigned long	__unused4;
};

#endif /* _S390_SEMBUF_H */
