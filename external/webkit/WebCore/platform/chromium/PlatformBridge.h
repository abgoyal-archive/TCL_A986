

#ifndef PlatformBridge_h
#define PlatformBridge_h

#include "ChromiumBridge.h"

namespace WebCore {

// FIXME: A better name for ChromiumBridge is PlatformBridge. Android already
// uses PlatformBridge so the code that is shared among the Android and Chromium
// ports is gradually moving towards using PlatformBridge. Once the Android
// unforking is done, we will change the name of ChromiumBridge to PlatformBridge
// and merge the two classes into one that will be shared by both ports.
typedef ChromiumBridge PlatformBridge;

} // namespace WebCore

#endif // PlatformBridge_h
