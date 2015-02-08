

#ifndef WebScreenInfoFactory_h
#define WebScreenInfoFactory_h

#include "../WebCommon.h"

#ifdef __OBJC__
@class NSView;
#else
class NSView;
#endif

namespace WebKit {

struct WebScreenInfo;

class WebScreenInfoFactory {
public:
    WEBKIT_API static WebScreenInfo screenInfo(NSView*);
};

} // namespace WebKit

#endif
