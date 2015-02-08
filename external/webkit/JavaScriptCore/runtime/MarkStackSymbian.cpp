

#include "config.h"
#include "MarkStack.h"

#if OS(SYMBIAN)

#include <e32hal.h>

namespace JSC {

void MarkStack::initializePagesize()
{
    TInt page_size;
    UserHal::PageSizeInBytes(page_size);
    MarkStack::s_pageSize = page_size;
}

void* MarkStack::allocateStack(size_t size)
{
    return fastMalloc(size);
}

void MarkStack::releaseStack(void* addr, size_t size)
{
    return fastFree(addr);
}

}

#endif
