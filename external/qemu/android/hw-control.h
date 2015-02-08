
#ifndef _android_hw_control_h
#define _android_hw_control_h

#include "qemu-common.h"

typedef void  (*AndroidHwLightBrightnessFunc)( void*       opaque,
                                               const char* light,
                                               int         brightness );

/* used to record a hw control 'client' */
typedef struct {
    AndroidHwLightBrightnessFunc  light_brightness;
} AndroidHwControlFuncs;

/* used to initialize the hardware control support */
extern void  android_hw_control_init( void*                         opaque,
                                      const AndroidHwControlFuncs*  funcs );

#endif /* _android_hw_control_h */
