

#ifndef WTF_StringExtras_h
#define WTF_StringExtras_h

#include <stdarg.h>
#include <stdio.h>

#if HAVE(STRINGS_H) 
#include <strings.h> 
#endif 

#if COMPILER(MSVC)
// FIXME: why a COMPILER check instead of OS? also, these should be HAVE checks

inline int snprintf(char* buffer, size_t count, const char* format, ...) 
{
    int result;
    va_list args;
    va_start(args, format);
    result = _vsnprintf(buffer, count, format, args);
    va_end(args);
    return result;
}

#if COMPILER(MSVC7) || OS(WINCE)

inline int vsnprintf(char* buffer, size_t count, const char* format, va_list args)
{
    return _vsnprintf(buffer, count, format, args);
}

#endif

#if OS(WINCE)

inline int strnicmp(const char* string1, const char* string2, size_t count)
{
    return _strnicmp(string1, string2, count);
}

inline int stricmp(const char* string1, const char* string2)
{
    return _stricmp(string1, string2);
}

inline char* strdup(const char* strSource)
{
    return _strdup(strSource);
}

#endif

inline int strncasecmp(const char* s1, const char* s2, size_t len)
{
    return _strnicmp(s1, s2, len);
}

inline int strcasecmp(const char* s1, const char* s2)
{
    return _stricmp(s1, s2);
}

#endif

#if OS(WINDOWS) || OS(LINUX) || OS(SOLARIS)
// FIXME: should check HAVE_STRNSTR

inline char* strnstr(const char* buffer, const char* target, size_t bufferLength)
{
    size_t targetLength = strlen(target);
    if (targetLength == 0)
        return const_cast<char*>(buffer);
    for (const char* start = buffer; *start && start + targetLength <= buffer + bufferLength; start++) {
        if (*start == *target && strncmp(start + 1, target + 1, targetLength - 1) == 0)
            return const_cast<char*>(start);
    }
    return 0;
}

#endif

#if COMPILER(RVCT) && __ARMCC_VERSION < 400000

int strcasecmp(const char* s1, const char* s2);
int strncasecmp(const char* s1, const char* s2, size_t len);

#endif

#endif // WTF_StringExtras_h
