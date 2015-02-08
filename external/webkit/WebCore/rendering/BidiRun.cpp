

#include "config.h"
#include "BidiRun.h"
#include "InlineBox.h"
#include "RenderArena.h"
#include <wtf/RefCountedLeakCounter.h>

using namespace WTF;

namespace WebCore {

#ifndef NDEBUG
static RefCountedLeakCounter bidiRunCounter("BidiRun");

static bool inBidiRunDestroy;
#endif

void BidiRun::destroy()
{
#ifndef NDEBUG
    inBidiRunDestroy = true;
#endif
    RenderArena* renderArena = m_object->renderArena();
    delete this;
#ifndef NDEBUG
    inBidiRunDestroy = false;
#endif

    // Recover the size left there for us by operator delete and free the memory.
    renderArena->free(*reinterpret_cast<size_t*>(this), this);
}

void* BidiRun::operator new(size_t sz, RenderArena* renderArena) throw()
{
#ifndef NDEBUG
    bidiRunCounter.increment();
#endif
    return renderArena->allocate(sz);
}

void BidiRun::operator delete(void* ptr, size_t sz)
{
#ifndef NDEBUG
    bidiRunCounter.decrement();
#endif
    ASSERT(inBidiRunDestroy);

    // Stash size where destroy() can find it.
    *(size_t*)ptr = sz;
}

}
