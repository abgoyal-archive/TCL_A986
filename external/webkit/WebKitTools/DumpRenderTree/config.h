

#define Config_H

#if defined(HAVE_CONFIG_H) && HAVE_CONFIG_H
#include "autotoolsconfig.h"
#endif

#include <wtf/Platform.h>

#ifdef __cplusplus
#undef new
#undef delete
#include <wtf/FastMalloc.h>
#endif

#if OS(WINDOWS) && !COMPILER(GCC)
#define JS_EXPORTDATA __declspec(dllimport)
#define WEBKIT_EXPORTDATA __declspec(dllimport)
#else
#define JS_EXPORTDATA
#define WEBKIT_EXPORTDATA
#endif

#if PLATFORM(WIN)
#define WTF_PLATFORM_CF 1 
#if defined(WIN_CAIRO)
#define WTF_PLATFORM_CAIRO 1
#define WTF_USE_CURL 1
#else
#define WTF_PLATFORM_CG 1
#define WTF_USE_CFNETWORK 1
#endif

#undef _WIN32_WINNT
#define _WIN32_WINNT 0x0500

#undef WINVER
#define WINVER 0x0500

// If we don't define these, they get defined in windef.h. 
// We want to use std::min and std::max
#undef max
#define max max
#undef min
#define min min

#undef _WINSOCKAPI_
#define _WINSOCKAPI_ // Prevent inclusion of winsock.h in windows.h
#endif  // PLATFORM(WIN)
