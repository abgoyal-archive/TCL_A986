

#ifndef __HIWEBCARBONUTILS__
#define __HIWEBCARBONUTILS__

#ifndef __LP64__

// These functions are only available for 32-bit.

#include <JavaScriptCore/WebKitAvailability.h>

#ifdef __OBJC__
#import <ApplicationServices/ApplicationServices.h>
@class NSImage;
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern void
WebInitForCarbon(void) AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_4_0;

#ifdef __OBJC__

extern CGImageRef
WebConvertNSImageToCGImageRef(NSImage * inImage) AVAILABLE_WEBKIT_VERSION_1_0_AND_LATER_BUT_DEPRECATED_IN_WEBKIT_VERSION_4_0;

#endif

#ifdef __cplusplus
}
#endif

#endif
#endif // __HIWEBCARBONUTILS__
