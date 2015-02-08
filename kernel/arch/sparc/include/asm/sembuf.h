
#ifndef _SPARC_SEMBUF_H
#define _SPARC_SEMBUF_H

#if defined(__sparc__) && defined(__arch64__)
# define PADDING(x)
#else
# define PADDING(x) unsigned int x;
#endif

struct semid64_ds {
	struct ipc64_perm sem_perm;		/* permissions .. see ipc.h */
	PADDING(__pad1)
	__kernel_time_t	sem_otime;		/* last semop time */
	PADDING(__pad2)
	__kernel_time_t	sem_ctime;		/* last change time */
	unsigned long	sem_nsems;		/* no. of semaphores in array */
	unsigned long	__unused1;
	unsigned long	__unused2;
};
#undef PADDING

#endif /* _SPARC64_SEMBUF_H */
