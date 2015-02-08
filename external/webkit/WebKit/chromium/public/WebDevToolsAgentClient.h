

#ifndef WebDevToolsAgentClient_h
#define WebDevToolsAgentClient_h

#include "WebCString.h"
#include "WebCommon.h"

namespace WebKit {
class WebString;
struct WebDevToolsMessageData;

class WebDevToolsAgentClient {
public:
    virtual void sendMessageToFrontend(const WebDevToolsMessageData&) { }

    // Invalidates widget which leads to the repaint.
    virtual void forceRepaint() { }

    // Returns the identifier of the entity hosting this agent.
    virtual int hostIdentifier() { return -1; }

    // Notifies host upon runtime feature being enabled/disabled.
    virtual void runtimeFeatureStateChanged(const WebString& feature, bool enabled) { }

    WEBKIT_API static void sendMessageToFrontendOnIOThread(const WebDevToolsMessageData&);

    virtual WebCString injectedScriptSource() { return WebCString(); }
    virtual WebCString injectedScriptDispatcherSource() { return WebCString(); }

protected:
    ~WebDevToolsAgentClient() { }
};

} // namespace WebKit

#endif
