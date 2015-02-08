
#ifndef _ASM_ALPHA_PARAM_H
#define _ASM_ALPHA_PARAM_H


#ifdef __KERNEL__
#define HZ		CONFIG_HZ
#define USER_HZ		HZ
#else
#define HZ		1024
#endif

#define EXEC_PAGESIZE	8192

#ifndef NOGROUP
#define NOGROUP		(-1)
#endif

#define MAXHOSTNAMELEN	64	/* max length of hostname */

#ifdef __KERNEL__
# define CLOCKS_PER_SEC	HZ	/* frequency at which times() counts */
#endif

#endif /* _ASM_ALPHA_PARAM_H */
