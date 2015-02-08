

#ifndef WebCommon_h
#define WebCommon_h

// -----------------------------------------------------------------------------
// Default configuration

#if !defined(WEBKIT_IMPLEMENTATION)
    #define WEBKIT_IMPLEMENTATION 0
#endif

#if !defined(WEBKIT_USING_SKIA)
    #if !defined(__APPLE__)
        #define WEBKIT_USING_SKIA 1
    #else
        #define WEBKIT_USING_SKIA 0
    #endif
#endif

#if !defined(WEBKIT_USING_CG)
    #if defined(__APPLE__)
        #define WEBKIT_USING_CG 1
    #else
        #define WEBKIT_USING_CG 0
    #endif
#endif

#if !defined(WEBKIT_USING_V8)
    #define WEBKIT_USING_V8 1
#endif

#if !defined(WEBKIT_USING_JSC)
    #define WEBKIT_USING_JSC 0
#endif

// -----------------------------------------------------------------------------
// Exported symbols need to be annotated with WEBKIT_API

#if defined(WIN32) && defined(WEBKIT_DLL)
    #if defined(WEBKIT_IMPLEMENTATION)
        #define WEBKIT_API __declspec(dllexport)
    #else
        #define WEBKIT_API __declspec(dllimport)
    #endif
#else
    #define WEBKIT_API
#endif

// -----------------------------------------------------------------------------
// Basic types

#include <stddef.h> // For size_t

namespace WebKit {

    // UTF-16 character type
#if defined(WIN32)
typedef wchar_t WebUChar;
#else
typedef unsigned short WebUChar;
#endif

} // namespace WebKit

#endif
