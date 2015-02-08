

#ifndef __W1_LOG_H
#define __W1_LOG_H

#define DEBUG

#ifdef W1_DEBUG
#  define assert(expr) do {} while (0)
#else
#  define assert(expr) \
        if(unlikely(!(expr))) {				        \
        printk(KERN_ERR "Assertion failed! %s,%s,%s,line=%d\n",	\
	#expr, __FILE__, __func__, __LINE__);		        \
        }
#endif

#endif /* __W1_LOG_H */

