

#include "config.h"
#include "HistoryItem.h"

namespace WebCore {

class AndroidWebHistoryBridge;

AndroidWebHistoryBridge* HistoryItem::bridge() const
{
    if (!m_bridge)
        return 0;
    return m_bridge.get();
}

void HistoryItem::setBridge(AndroidWebHistoryBridge* bridge)
{
    m_bridge = adoptRef(bridge);
}

} // namespace WebCore

