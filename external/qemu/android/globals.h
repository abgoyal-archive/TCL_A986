
#ifndef _ANDROID_GLOBALS_H
#define _ANDROID_GLOBALS_H

#include "android/avd/info.h"
#include "android/avd/hw-config.h"

extern AvdInfoParams     android_avdParams[1];

extern AvdInfo*          android_avdInfo;

/* the hardware configuration for this specific virtual device */
extern AndroidHwConfig   android_hw[1];

#endif /* _ANDROID_GLOBALS_H */
