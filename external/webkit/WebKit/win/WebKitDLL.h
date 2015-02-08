

#ifndef WebKitDLL_H
#define WebKitDLL_H

#ifndef _HAS_EXCEPTIONS
#define _HAS_EXCEPTIONS 0
#endif

#include <winsock2.h>
#include <windows.h>
#include <wtf/HashCountedSet.h>
#include <WebCore/PlatformString.h>
#include <WebCore/StringHash.h>

#ifdef WEBKIT_EXPORTS
#define WEBKIT_API __declspec(dllexport)
#else
#define WEBKIT_API __declspec(dllimport)
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern ULONG gLockCount;
extern ULONG gClassCount;
extern HashCountedSet<WebCore::String> gClassNameCount;
extern HINSTANCE gInstance;
extern CLSID gRegCLSIDs[];

#ifdef __cplusplus
}
#endif

#endif // WebKitDLL_H
