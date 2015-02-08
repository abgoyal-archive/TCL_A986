

#include "config.h"
#include "NetworkStateNotifier.h"

namespace WebCore {

void NetworkStateNotifier::networkStateChange(bool online)
{   
    if (m_isOnLine == online)
        return;
    
    m_isOnLine = online;

    if (m_networkStateChangedFunction)
        m_networkStateChangedFunction();
}

// TODO: Upstream to webkit.org
void NetworkStateNotifier::networkTypeChange(Connection::ConnectionType type)
{
    if (m_type == type)
        return;

    m_type = type;

    if (m_networkStateChangedFunction)
        m_networkStateChangedFunction();
}

}
