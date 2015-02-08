

#include "config.h"
#include "ChildThreadDOMData.h"

namespace WebCore {

ChildThreadDOMData::ChildThreadDOMData()
    : m_defaultStore(this)
{
}

DOMDataStore& ChildThreadDOMData::getStore()
{
    ASSERT(!WTF::isMainThread());
    // Currently, child threads have only one world.
    return m_defaultStore;
}

} // namespace WebCore
