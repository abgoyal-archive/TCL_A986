

#ifndef DebuggerAgentManager_h
#define DebuggerAgentManager_h

#include "WebDevToolsAgent.h"
#include <v8-debug.h>
#include <wtf/HashMap.h>
#include <wtf/Noncopyable.h>

namespace WebCore {
class PageGroupLoadDeferrer;
class String;
}

namespace WebKit {

class DebuggerAgentImpl;
class DictionaryValue;
class WebFrameImpl;
class WebViewImpl;

// There is single v8 instance per render process. Also there may be several
// RenderViews and consequently devtools agents in the process that want to talk
// to the v8 debugger. This class coordinates communication between the debug
// agents and v8 debugger. It will set debug output handler as long as at least
// one debugger agent is attached and remove it when last debugger agent is
// detached. When message is received from debugger it will route it to the
// right debugger agent if there is one otherwise the message will be ignored.
//
// v8 may send a message(e.g. exception event) after which it
// would expect some actions from the handler. If there is no appropriate
// debugger agent to handle such messages the manager will perform the action
// itself, otherwise v8 may hang waiting for the action.
class DebuggerAgentManager : public Noncopyable {
public:
    static void debugAttach(DebuggerAgentImpl* debuggerAgent);
    static void debugDetach(DebuggerAgentImpl* debuggerAgent);
    static void pauseScript();
    static void executeDebuggerCommand(const WebCore::String& command, int callerId);
    static void setMessageLoopDispatchHandler(WebDevToolsAgent::MessageLoopDispatchHandler handler);

    // Sets |hostId| as the frame context data. This id is used to filter scripts
    // related to the inspected page.
    static void setHostId(WebFrameImpl* webframe, int hostId);

    static void onWebViewClosed(WebViewImpl* webview);

    static void onNavigate();

    class UtilityContextScope : public Noncopyable {
    public:
        UtilityContextScope()
        {
            ASSERT(!s_inUtilityContext);
            s_inUtilityContext = true;
        }
        ~UtilityContextScope()
        {
            if (s_debugBreakDelayed) {
              v8::Debug::DebugBreak();
              s_debugBreakDelayed = false;
            }
            s_inUtilityContext = false;
        }
    };

private:
    DebuggerAgentManager();
    ~DebuggerAgentManager();

    static void debugHostDispatchHandler();
    static void onV8DebugMessage(const v8::Debug::Message& message);
    static void sendCommandToV8(const WebCore::String& cmd,
                                v8::Debug::ClientData* data);
    static void sendContinueCommandToV8();

    static DebuggerAgentImpl* findAgentForCurrentV8Context();
    static DebuggerAgentImpl* debuggerAgentForHostId(int hostId);

    typedef HashMap<int, DebuggerAgentImpl*> AttachedAgentsMap;
    static AttachedAgentsMap* s_attachedAgentsMap;

    static WebDevToolsAgent::MessageLoopDispatchHandler s_messageLoopDispatchHandler;
    static bool s_inHostDispatchHandler;
    typedef HashMap<WebViewImpl*, WebCore::PageGroupLoadDeferrer*> DeferrersMap;
    static DeferrersMap s_pageDeferrers;

    static bool s_inUtilityContext;
    static bool s_debugBreakDelayed;
};

} // namespace WebKit

#endif
