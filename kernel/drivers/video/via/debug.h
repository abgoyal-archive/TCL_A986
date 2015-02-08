
#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifndef VIAFB_DEBUG
#define VIAFB_DEBUG 0
#endif

#if VIAFB_DEBUG
#define DEBUG_MSG(f, a...)   printk(f, ## a)
#else
#define DEBUG_MSG(f, a...)
#endif

#define VIAFB_WARN 0
#if VIAFB_WARN
#define WARN_MSG(f, a...)   printk(f, ## a)
#else
#define WARN_MSG(f, a...)
#endif

#endif /* __DEBUG_H__ */
