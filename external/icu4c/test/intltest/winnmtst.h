
#ifndef __WINNMTST
#define __WINNMTST

#include "unicode/utypes.h"

#ifdef U_WINDOWS

#if !UCONFIG_NO_FORMATTING


class TestLog;

class Win32NumberTest
{
public:
    static void testLocales(TestLog *log);

private:
    Win32NumberTest();
};

#endif /* #if !UCONFIG_NO_FORMATTING */

#endif // #ifdef U_WINDOWS

#endif // __WINNMTST
