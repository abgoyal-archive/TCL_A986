

#ifndef ToolsAgent_h
#define ToolsAgent_h

#include "DevToolsRPC.h"

namespace WebKit {

// Tools agent provides API for enabling / disabling other agents as well as
// API for auxiliary UI functions such as dom elements highlighting.
#define TOOLS_AGENT_STRUCT(METHOD0, METHOD1, METHOD2, METHOD3, METHOD4, METHOD5) \
    /* Dispatches given function on the InspectorController object */ \
    METHOD3(dispatchOnInspectorController, int /* call_id */, \
        String /* function_name */, String /* json_args */) \
    \
    /* Dispatches given function on the InjectedScript object */ \
    METHOD5(dispatchOnInjectedScript, int /* call_id */, \
        int /* injected_script_id */, String /* function_name */, \
        String /* json_args */, bool /* async */)

DEFINE_RPC_CLASS(ToolsAgent, TOOLS_AGENT_STRUCT)

#define TOOLS_AGENT_DELEGATE_STRUCT(METHOD0, METHOD1, METHOD2, METHOD3, METHOD4, METHOD5) \
    /* Updates focused node on the client. */ \
    METHOD1(frameNavigate, String /* url */) \
    \
    /* Response to the DispatchOn*. */ \
    METHOD3(didDispatchOn, int /* call_id */, String /* result */, String /* exception */) \
    \
    /* Sends InspectorFrontend message to be dispatched on client. */ \
    METHOD1(dispatchOnClient, String /* data */)

DEFINE_RPC_CLASS(ToolsAgentDelegate, TOOLS_AGENT_DELEGATE_STRUCT)

} // namespace WebKit

#endif
