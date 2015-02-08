

#include "config.h"
#include "MainThreadDOMData.h"

#include "V8IsolatedContext.h"

namespace WebCore {

MainThreadDOMData::MainThreadDOMData()
    : m_defaultStore(this)
{
}
    
MainThreadDOMData* MainThreadDOMData::getCurrent()
{
    ASSERT(WTF::isMainThread());
    DEFINE_STATIC_LOCAL(MainThreadDOMData, mainThreadDOMData, ());
    return &mainThreadDOMData;
}

DOMDataStore& MainThreadDOMData::getMainThreadStore()
{
    // This is broken out as a separate non-virtual method from getStore()
    // so that it can be inlined by getCurrentMainThreadStore, which is
    // a hot spot in Dromaeo DOM tests.
    ASSERT(WTF::isMainThread());
    V8IsolatedContext* context = V8IsolatedContext::getEntered();
    if (UNLIKELY(context != 0))
        return *context->world()->domDataStore();
    return m_defaultStore;
}

DOMDataStore& MainThreadDOMData::getCurrentMainThreadStore()
{
    return getCurrent()->getMainThreadStore();
}


} // namespace WebCore
