

#include "config.h"
#include "SystemTime.h"

#include <CoreGraphics/CGEventSource.h>
#include <CoreFoundation/CFDate.h>

namespace WebCore {

float userIdleTime()
{
    return static_cast<float>(CGEventSourceSecondsSinceLastEventType(kCGEventSourceStateCombinedSessionState, kCGAnyInputEventType));
}

}
