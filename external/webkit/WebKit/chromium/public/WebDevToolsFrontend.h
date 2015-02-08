

#ifndef WebDevToolsFrontend_h
#define WebDevToolsFrontend_h

#include "WebCommon.h"

namespace WebKit {

class WebDevToolsFrontendClient;
class WebString;
class WebView;
struct WebDevToolsMessageData;

// WebDevToolsFrontend represents DevTools client sitting in the Glue. It provides
// direct and delegate Apis to the host.
class WebDevToolsFrontend {
public:
    WEBKIT_API static WebDevToolsFrontend* create(WebView* view,
                                                  WebDevToolsFrontendClient* client,
                                                  const WebString& applicationLocale);

    virtual ~WebDevToolsFrontend() {}

    virtual void dispatchMessageFromAgent(const WebDevToolsMessageData&) = 0;
};

} // namespace WebKit

#endif
