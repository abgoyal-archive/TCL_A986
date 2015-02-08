

#ifndef DebuggerAgentImpl_h
#define DebuggerAgentImpl_h

#include "DebuggerAgent.h"

#include <v8.h>
#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>

namespace WebCore {
class Document;
class Frame;
class Node;
class Page;
class String;
}

namespace WebKit {

class WebDevToolsAgentImpl;
class WebViewImpl;

class DebuggerAgentImpl : public DebuggerAgent {
public:
    // Creates utility context with injected js agent.
    static void createUtilityContext(WebCore::Frame* frame, v8::Persistent<v8::Context>* context);

    DebuggerAgentImpl(WebKit::WebViewImpl* webViewImpl,
                      DebuggerAgentDelegate* delegate,
                      WebDevToolsAgentImpl* webdevtoolsAgent);
    virtual ~DebuggerAgentImpl();

    // DebuggerAgent implementation.
    virtual void getContextId();
    virtual void processDebugCommands();

    void debuggerOutput(const WebCore::String& out);

    void setAutoContinueOnException(bool autoContinue) { m_autoContinueOnException = autoContinue; }

    bool autoContinueOnException() { return m_autoContinueOnException; }

    // Executes function with the given name in the utility context. Passes node
    // and json args as parameters. Note that the function called must be
    // implemented in the inject_dispatch.js file.
    WebCore::String executeUtilityFunction(
        v8::Handle<v8::Context> context,
        int callId,
        const char* object,
        const WebCore::String& functionName,
        const WebCore::String& jsonArgs,
        bool async,
        WebCore::String* exception);


    WebCore::Page* page();
    WebDevToolsAgentImpl* webdevtoolsAgent() { return m_webdevtoolsAgent; }

    WebKit::WebViewImpl* webView() { return m_webViewImpl; }

private:
    WebKit::WebViewImpl* m_webViewImpl;
    DebuggerAgentDelegate* m_delegate;
    WebDevToolsAgentImpl* m_webdevtoolsAgent;
    bool m_autoContinueOnException;
};

} // namespace WebKit

#endif
