

#include "config.h"
#include "NetworkStateNotifier.h"

namespace WebCore {

// Chromium doesn't currently support network state notifications. This causes
// an extra DLL to get loaded into the renderer which can slow things down a
// bit. We may want an alternate design.

void NetworkStateNotifier::updateState()
{
}

NetworkStateNotifier::NetworkStateNotifier()
    : m_isOnLine(true)
    , m_networkStateChangedFunction(0)
{
}

} // namespace WebCore
