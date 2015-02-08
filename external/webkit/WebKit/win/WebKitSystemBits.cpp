

#include "config.h"
#include "WebKitSystemBits.h"

#pragma warning(push, 0)
#include <WebCore/PlatformString.h>
#pragma warning(pop)

#include <windows.h>
#include <CoreFoundation/CoreFoundation.h>

unsigned long long WebMemorySize()
{
    MEMORYSTATUSEX statex;
    statex.dwLength = sizeof(statex);
    GlobalMemoryStatusEx(&statex);
    return statex.ullTotalPhys;
}

unsigned long long WebVolumeFreeSize(CFStringRef cfstringPath)
{
    WebCore::String path(cfstringPath);
    ULARGE_INTEGER freeBytesToCaller;
    BOOL result = GetDiskFreeSpaceExW((LPCWSTR)path.charactersWithNullTermination(), &freeBytesToCaller, 0, 0);
    if (!result)
        return 0;

    return freeBytesToCaller.QuadPart;
}
