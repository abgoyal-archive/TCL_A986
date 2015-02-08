

#ifndef ParserUtilities_h
#define ParserUtilities_h

#include "PlatformString.h"

namespace WebCore {

    inline bool skipString(const UChar*& ptr, const UChar* end, const UChar* name, int length)
    {
        if (end - ptr < length)
            return false;
        if (memcmp(name, ptr, sizeof(UChar) * length))
            return false;
        ptr += length;
        return true;
    }

    inline bool skipString(const UChar*& ptr, const UChar* end, const char* str)
    {
        int length = strlen(str);
        if (end - ptr < length)
            return false;
        for (int i = 0; i < length; ++i) {
            if (ptr[i] != str[i])
                return false;
        }
        ptr += length;
        return true;
    }

} // namspace WebCore

#endif // ParserUtilities_h
