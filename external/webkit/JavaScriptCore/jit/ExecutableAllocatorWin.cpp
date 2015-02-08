

#include "config.h"

#include "ExecutableAllocator.h"

#if ENABLE(ASSEMBLER) && OS(WINDOWS)

#include "windows.h"

namespace JSC {

void ExecutableAllocator::intializePageSize()
{
    SYSTEM_INFO system_info;
    GetSystemInfo(&system_info);
    ExecutableAllocator::pageSize = system_info.dwPageSize;
}

ExecutablePool::Allocation ExecutablePool::systemAlloc(size_t n)
{
    void* allocation = VirtualAlloc(0, n, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!allocation)
        CRASH();
    ExecutablePool::Allocation alloc = {reinterpret_cast<char*>(allocation), n};
    return alloc;
}

void ExecutablePool::systemRelease(const ExecutablePool::Allocation& alloc)
{ 
    VirtualFree(alloc.pages, 0, MEM_RELEASE); 
}

#if ENABLE(ASSEMBLER_WX_EXCLUSIVE)
#error "ASSEMBLER_WX_EXCLUSIVE not yet suported on this platform."
#endif

}

#endif // HAVE(ASSEMBLER)
