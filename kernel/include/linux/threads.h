
#ifndef _LINUX_THREADS_H
#define _LINUX_THREADS_H



#ifndef CONFIG_NR_CPUS
/* FIXME: This should be fixed in the arch's Kconfig */
#define CONFIG_NR_CPUS	1
#endif

/* Places which use this should consider cpumask_var_t. */
#define NR_CPUS		CONFIG_NR_CPUS

#define MIN_THREADS_LEFT_FOR_ROOT 4

#define PID_MAX_DEFAULT (CONFIG_BASE_SMALL ? 0x1000 : 0x8000)

#define PID_MAX_LIMIT (CONFIG_BASE_SMALL ? PAGE_SIZE * 8 : \
	(sizeof(long) > 4 ? 4 * 1024 * 1024 : PID_MAX_DEFAULT))

#define PIDS_PER_CPU_DEFAULT	1024
#define PIDS_PER_CPU_MIN	8

#endif
