
#ifndef _ANDROID_USER_CONFIG_H
#define _ANDROID_USER_CONFIG_H

#include "android/avd/info.h"
#include <stdint.h>

typedef struct AUserConfig   AUserConfig;

/* Create a new AUserConfig object from a given AvdInfo */
AUserConfig*   auserConfig_new( AvdInfo*  info );

/* Retrieve the unique UID for this AVD */
uint64_t       auserConfig_getUUID( AUserConfig*  uconfig );

/* Retrieve the stored window position for this AVD */
void           auserConfig_getWindowPos( AUserConfig*  uconfig, int  *pX, int  *pY );

/* Change the stored window position for this AVD */
void           auserConfig_setWindowPos( AUserConfig*  uconfig, int  x, int  y );

void           auserConfig_save( AUserConfig*  uconfig );

/* */

#endif /* _ANDROID_USER_CONFIG_H */
