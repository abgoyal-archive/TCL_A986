

#include "config.h"
#include "DNS.h"

#include "ChromiumBridge.h"

namespace WebCore {

void prefetchDNS(const String& hostname)
{
    ChromiumBridge::prefetchDNS(hostname);
}

} // namespace WebCore
