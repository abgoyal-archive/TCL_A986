

#ifndef DCOOKIES_H
#define DCOOKIES_H
 

#ifdef CONFIG_PROFILING
 
#include <linux/dcache.h>
#include <linux/path.h>
#include <linux/types.h>
 
struct dcookie_user;
 
struct dcookie_user * dcookie_register(void);

void dcookie_unregister(struct dcookie_user * user);
  
int get_dcookie(struct path *path, unsigned long *cookie);

#else

static inline struct dcookie_user * dcookie_register(void)
{
	return NULL;
}

static inline void dcookie_unregister(struct dcookie_user * user)
{
	return;
}

static inline int get_dcookie(struct path *path, unsigned long *cookie)
{
	return -ENOSYS;
}

#endif /* CONFIG_PROFILING */

#endif /* DCOOKIES_H */
