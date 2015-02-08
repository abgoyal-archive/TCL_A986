

#ifndef WebDevToolsFrontendClient_h
#define WebDevToolsFrontendClient_h

namespace WebKit {

class WebString;
struct WebDevToolsMessageData;

class WebDevToolsFrontendClient {
public:
    WebDevToolsFrontendClient() {}

    virtual void sendMessageToAgent(const WebDevToolsMessageData&) {};
    virtual void sendDebuggerCommandToAgent(const WebString& command) {};
    virtual void sendDebuggerPauseScript() {}

    virtual void activateWindow() {};
    virtual void closeWindow() {};
    virtual void dockWindow() {};
    virtual void undockWindow() {};

protected:
    virtual ~WebDevToolsFrontendClient() {}
};

} // namespace WebKit

#endif
