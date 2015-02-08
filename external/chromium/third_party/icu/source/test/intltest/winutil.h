
#ifndef __WINUTIL
#define __WINUTIL

#include "unicode/utypes.h"

#ifdef U_WINDOWS

#if !UCONFIG_NO_FORMATTING


class Win32Utilities
{
public:
    struct LCIDRecord
    {
        int32_t lcid;
        const char *localeID;
    };

    static LCIDRecord *getLocales(int32_t &localeCount);
    static void freeLocales(LCIDRecord *records);

private:
    Win32Utilities();
};

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // #ifdef U_WINDOWS

#endif // __WINUTIL
