
#ifndef __WINDTTST
#define __WINDTTST

#include "unicode/utypes.h"

#ifdef U_WINDOWS

#if !UCONFIG_NO_FORMATTING


class TestLog;

class Win32DateTimeTest
{
public:
    static void testLocales(TestLog *log);

private:
    Win32DateTimeTest();
};

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // #ifdef U_WINDOWS

#endif // __WINDTTST
