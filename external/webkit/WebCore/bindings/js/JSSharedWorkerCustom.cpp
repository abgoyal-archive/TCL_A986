

#include "config.h"

#if ENABLE(SHARED_WORKERS)

#include "JSSharedWorker.h"

#include "JSDOMGlobalObject.h"
#include "SharedWorker.h"

using namespace JSC;

namespace WebCore {

void JSSharedWorker::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    if (MessagePort* port = impl()->port())
        markDOMObjectWrapper(markStack, *Heap::heap(this)->globalData(), port);
}

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)
