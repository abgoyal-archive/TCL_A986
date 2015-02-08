

#ifndef WebScreenInfoFactory_h
#define WebScreenInfoFactory_h

#include "../WebCommon.h"

typedef struct _XDisplay Display;

namespace WebKit {

struct WebScreenInfo;

class WebScreenInfoFactory {
public:
    WEBKIT_API static WebScreenInfo screenInfo(Display*, int screenNumber);
};

} // namespace WebKit

#endif
