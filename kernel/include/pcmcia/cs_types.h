

#ifndef _LINUX_CS_TYPES_H
#define _LINUX_CS_TYPES_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <sys/types.h>
#endif

typedef u_short	socket_t;
typedef u_int	event_t;
typedef u_char	cisdata_t;
typedef u_short	page_t;

typedef unsigned long window_handle_t;

struct region_t;
typedef struct region_t *memory_handle_t;

#ifndef DEV_NAME_LEN
#define DEV_NAME_LEN 32
#endif

typedef char dev_info_t[DEV_NAME_LEN];

#endif /* _LINUX_CS_TYPES_H */
