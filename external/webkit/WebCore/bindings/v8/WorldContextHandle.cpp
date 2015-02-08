

#include "config.h"
#include "WorldContextHandle.h"

#include "V8IsolatedContext.h"

namespace WebCore {

WorldContextHandle::WorldContextHandle(WorldToUse worldToUse)
    : m_worldToUse(worldToUse)
{
    if (worldToUse == UseMainWorld)
        return;

    if (V8IsolatedContext* context = V8IsolatedContext::getEntered())
        m_context = context->sharedContext();
}

v8::Local<v8::Context> WorldContextHandle::adjustedContext(V8Proxy* proxy) const
{
    if (m_worldToUse == UseMainWorld)
        return proxy->mainWorldContext();
    if (!m_context || m_context->get().IsEmpty())
        return proxy->context();
    return v8::Local<v8::Context>::New(m_context->get());
}

} // namespace WebCore
