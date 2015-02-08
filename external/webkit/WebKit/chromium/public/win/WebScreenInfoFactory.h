

#ifndef WebScreenInfoFactory_h
#define WebScreenInfoFactory_h

#include "../WebCommon.h"

typedef struct HWND__* HWND;

namespace WebKit {

struct WebScreenInfo;

class WebScreenInfoFactory {
public:
    WEBKIT_API static WebScreenInfo screenInfo(HWND);
};

} // namespace WebKit

#endif
