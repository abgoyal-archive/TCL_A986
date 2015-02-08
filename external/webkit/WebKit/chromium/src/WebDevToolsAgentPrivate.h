

#ifndef WebDevToolsAgentPrivate_h
#define WebDevToolsAgentPrivate_h

// FIXME: This relative path is a temporary hack to support using this
// header from webkit/glue.
#include "../public/WebDevToolsAgent.h"

namespace WebKit {
class WebFrameImpl;

class WebDevToolsAgentPrivate : public WebDevToolsAgent {
public:
    // Notifications from FrameLoaderClientImpl:

    // The window object for the frame has been cleared of any extra properties
    // that may have been set by script from the previously loaded document.
    virtual void didClearWindowObject(WebFrameImpl*) = 0;

    // The provisional datasource is now committed.  The first part of the
    // response body has been received, and the encoding of the response body
    // is known.
    virtual void didCommitProvisionalLoad(WebFrameImpl*, bool isNewNavigation) = 0;
};

} // namespace WebKit

#endif
