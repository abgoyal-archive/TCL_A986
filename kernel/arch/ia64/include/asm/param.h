
#ifndef _ASM_IA64_PARAM_H
#define _ASM_IA64_PARAM_H


#define EXEC_PAGESIZE	65536

#ifndef NOGROUP
# define NOGROUP	(-1)
#endif

#define MAXHOSTNAMELEN	64	/* max length of hostname */

#ifdef __KERNEL__
# define HZ		CONFIG_HZ
# define USER_HZ	HZ
# define CLOCKS_PER_SEC	HZ	/* frequency at which times() counts */
#else
   /*
    * Technically, this is wrong, but some old apps still refer to it.  The proper way to
    * get the HZ value is via sysconf(_SC_CLK_TCK).
    */
# define HZ 1024
#endif

#endif /* _ASM_IA64_PARAM_H */
