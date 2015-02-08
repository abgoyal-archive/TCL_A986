

#include "config.h"
#include "SystemTime.h"

#include "ChromiumBridge.h"
#include "NotImplemented.h"

namespace WebCore {

// Get the current time in seconds since epoch.
double currentTime()
{
    return ChromiumBridge::currentTime();
}

float userIdleTime()
{
    // Needed for back/forward cache, which we currently have disabled.
    notImplemented();
    return 0.0F;
}

} // namespace WebCore
