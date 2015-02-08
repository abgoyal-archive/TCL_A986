

#include "config.h"
#include "NetworkStateNotifier.h"

#include <wtf/Assertions.h>
#include <wtf/StdLibExtras.h>
#include <wtf/Threading.h>

namespace WebCore {

NetworkStateNotifier& networkStateNotifier()
{
    AtomicallyInitializedStatic(NetworkStateNotifier*, networkStateNotifier = new NetworkStateNotifier);
    
    return *networkStateNotifier;
}

void NetworkStateNotifier::setNetworkStateChangedFunction(void(*function)())
{
    ASSERT(!m_networkStateChangedFunction);
    
    m_networkStateChangedFunction = function;
}
    
}
