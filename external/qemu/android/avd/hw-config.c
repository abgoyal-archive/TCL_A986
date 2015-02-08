
#include "android/avd/hw-config.h"
#include "android/utils/ini.h"
#include <string.h>
#include <stdlib.h>


/* the global variable containing the hardware config for this device */
AndroidHwConfig   android_hw[1];

int
androidHwConfig_read( AndroidHwConfig*  config,
                      IniFile*          ini )
{
    if (ini == NULL)
        return -1;

    /* use the magic of macros to implement the hardware configuration loaded */

#define   HWCFG_BOOL(n,s,d,a,t)       config->n = iniFile_getBoolean(ini, s, d);
#define   HWCFG_INT(n,s,d,a,t)        config->n = iniFile_getInteger(ini, s, d);
#define   HWCFG_STRING(n,s,d,a,t)     config->n = iniFile_getString(ini, s, d);
#define   HWCFG_DOUBLE(n,s,d,a,t)     config->n = iniFile_getDouble(ini, s, d);
#define   HWCFG_DISKSIZE(n,s,d,a,t)   config->n = iniFile_getDiskSize(ini, s, d);

#include "android/avd/hw-config-defs.h"

    return 0;
}
