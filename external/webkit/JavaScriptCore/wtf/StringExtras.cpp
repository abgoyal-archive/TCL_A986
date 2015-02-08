

#include "config.h"

#if COMPILER(RVCT) && __ARMCC_VERSION < 400000

#include "StringExtras.h"

#include "ASCIICType.h"

int strcasecmp(const char* s1, const char* s2)
{
    while (toASCIIUpper(*s1) == toASCIIUpper(*s2)) {
        if (*s1 == '\0')
            return 0;
        s1++;
        s2++;
    }

    return toASCIIUpper(*s1) - toASCIIUpper(*s2);
}

int strncasecmp(const char* s1, const char* s2, size_t len)
{
    while (len > 0 && toASCIIUpper(*s1) == toASCIIUpper(*s2)) {
        if (*s1 == '\0')
            return 0;
        s1++;
        s2++;
        len--;
    }

    if (!len)
        return 0;

    return toASCIIUpper(*s1) - toASCIIUpper(*s2);
}

#endif
