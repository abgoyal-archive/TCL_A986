

#ifndef WTF_UNICODE_H
#define WTF_UNICODE_H

#include <wtf/Assertions.h>

#if USE(QT4_UNICODE)
#include "qt4/UnicodeQt4.h"
#elif USE(ICU_UNICODE)
#include <wtf/unicode/icu/UnicodeIcu.h>
#elif USE(GLIB_UNICODE)
#include <wtf/unicode/glib/UnicodeGLib.h>
#elif USE(WINCE_UNICODE)
#include <wtf/unicode/wince/UnicodeWince.h>
#else
#error "Unknown Unicode implementation"
#endif

COMPILE_ASSERT(sizeof(UChar) == 2, UCharIsTwoBytes);

#endif // WTF_UNICODE_H
