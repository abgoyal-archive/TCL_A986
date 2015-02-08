
#ifndef _android_kmsg_h
#define _android_kmsg_h

#include "qemu-common.h"


typedef enum {
    ANDROID_KMSG_SAVE_MESSAGES  = (1 << 0),
    ANDROID_KMSG_PRINT_MESSAGES = (1 << 1),
} AndroidKmsgFlags;

extern void  android_kmsg_init( AndroidKmsgFlags  flags );

extern CharDriverState*  android_kmsg_get_cs( void );

#endif /* _android_kmsg_h */
