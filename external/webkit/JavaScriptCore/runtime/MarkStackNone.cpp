

#include "config.h"

#include "MarkStack.h"

#include "FastMalloc.h"

namespace JSC {

void MarkStack::initializePagesize()
{
    MarkStack::s_pageSize = 4096;
}

void* MarkStack::allocateStack(size_t size)
{
    return fastMalloc(size);
}

void MarkStack::releaseStack(void* addr, size_t)
{
    return fastFree(addr);
}

}
