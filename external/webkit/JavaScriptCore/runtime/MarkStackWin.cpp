

#include "config.h"
#include "MarkStack.h"

#if OS(WINDOWS)

#include "windows.h"

namespace JSC {

void MarkStack::initializePagesize()
{
    SYSTEM_INFO system_info;
    GetSystemInfo(&system_info);
    MarkStack::s_pageSize = system_info.dwPageSize;
}

void* MarkStack::allocateStack(size_t size)
{
    return VirtualAlloc(0, size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
}
void MarkStack::releaseStack(void* addr, size_t)
{
    // According to http://msdn.microsoft.com/en-us/library/aa366892(VS.85).aspx,
    // dwSize must be 0 if dwFreeType is MEM_RELEASE.
    VirtualFree(addr, 0, MEM_RELEASE);
}

}

#endif
