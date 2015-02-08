
#ifndef _ANDROID_AVD_HW_CONFIG_H
#define _ANDROID_AVD_HW_CONFIG_H

#include <stdint.h>
#include "android/utils/ini.h"

typedef char      hw_bool_t;
typedef int       hw_int_t;
typedef int64_t   hw_disksize_t;
typedef char*     hw_string_t;
typedef double    hw_double_t;

#define   HWCFG_BOOL(n,s,d,a,t)       hw_bool_t      n;
#define   HWCFG_INT(n,s,d,a,t)        hw_int_t       n;
#define   HWCFG_STRING(n,s,d,a,t)     hw_string_t    n;
#define   HWCFG_DOUBLE(n,s,d,a,t)     hw_double_t    n;
#define   HWCFG_DISKSIZE(n,s,d,a,t)   hw_disksize_t  n;

typedef struct {
#include "android/avd/hw-config-defs.h"
} AndroidHwConfig;

int  androidHwConfig_read( AndroidHwConfig*  hwConfig,
                           IniFile*          configFile );

#endif /* _ANDROID_AVD_HW_CONFIG_H */
