

#include "config.h"
#include "JSMessageChannel.h"

#include "MessageChannel.h"

using namespace JSC;

namespace WebCore {
    
void JSMessageChannel::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    if (MessagePort* port = m_impl->port1())
        markDOMObjectWrapper(markStack, *Heap::heap(this)->globalData(), port);

    if (MessagePort* port = m_impl->port2())
        markDOMObjectWrapper(markStack, *Heap::heap(this)->globalData(), port);
}

} // namespace WebCore
