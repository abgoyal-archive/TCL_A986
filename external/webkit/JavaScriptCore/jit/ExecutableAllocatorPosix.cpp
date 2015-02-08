

#include "config.h"

#include "ExecutableAllocator.h"

#if ENABLE(ASSEMBLER) && OS(UNIX) && !OS(SYMBIAN)

#include <sys/mman.h>
#include <unistd.h>
#include <wtf/VMTags.h>

namespace JSC {

#if !(OS(DARWIN) && CPU(X86_64))

void ExecutableAllocator::intializePageSize()
{
    ExecutableAllocator::pageSize = getpagesize();
}

ExecutablePool::Allocation ExecutablePool::systemAlloc(size_t n)
{
    void* allocation = mmap(NULL, n, INITIAL_PROTECTION_FLAGS, MAP_PRIVATE | MAP_ANON, VM_TAG_FOR_EXECUTABLEALLOCATOR_MEMORY, 0);
    if (allocation == MAP_FAILED)
        CRASH();
    ExecutablePool::Allocation alloc = { reinterpret_cast<char*>(allocation), n };
    return alloc;
}

void ExecutablePool::systemRelease(const ExecutablePool::Allocation& alloc)
{ 
    int result = munmap(alloc.pages, alloc.size);
    ASSERT_UNUSED(result, !result);
}

#endif // !(OS(DARWIN) && CPU(X86_64))

#if ENABLE(ASSEMBLER_WX_EXCLUSIVE)
void ExecutableAllocator::reprotectRegion(void* start, size_t size, ProtectionSeting setting)
{
    if (!pageSize)
        intializePageSize();

    // Calculate the start of the page containing this region,
    // and account for this extra memory within size.
    intptr_t startPtr = reinterpret_cast<intptr_t>(start);
    intptr_t pageStartPtr = startPtr & ~(pageSize - 1);
    void* pageStart = reinterpret_cast<void*>(pageStartPtr);
    size += (startPtr - pageStartPtr);

    // Round size up
    size += (pageSize - 1);
    size &= ~(pageSize - 1);

    mprotect(pageStart, size, (setting == Writable) ? PROTECTION_FLAGS_RW : PROTECTION_FLAGS_RX);
}
#endif

}

#endif // HAVE(ASSEMBLER)
