

#include "config.h"
#include "MarkStack.h"

#if OS(UNIX) && !OS(SYMBIAN)

#include <unistd.h>
#include <sys/mman.h>

namespace JSC {

void MarkStack::initializePagesize()
{
    MarkStack::s_pageSize = getpagesize();
}

void* MarkStack::allocateStack(size_t size)
{
    return mmap(0, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANON, -1, 0);
}
void MarkStack::releaseStack(void* addr, size_t size)
{
    munmap(addr, size);
}

}

#endif
