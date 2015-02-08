#ifndef _LINUX_THREADS_H
#define _LINUX_THREADS_H



#ifdef CONFIG_SMP
#define NR_CPUS		CONFIG_NR_CPUS
#else
#define NR_CPUS		1
#endif

#define MIN_THREADS_LEFT_FOR_ROOT 4

#define PID_MAX_DEFAULT (CONFIG_BASE_SMALL ? 0x1000 : 0x8000)

#define PID_MAX_LIMIT (CONFIG_BASE_SMALL ? PAGE_SIZE * 8 : \
	(sizeof(long) > 4 ? 4 * 1024 * 1024 : PID_MAX_DEFAULT))

#endif
