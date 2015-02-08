

#ifndef DebuggerAgent_h
#define DebuggerAgent_h

#include "DevToolsRPC.h"

namespace WebKit {

#define DEBUGGER_AGENT_STRUCT(METHOD0, METHOD1, METHOD2, METHOD3, METHOD4, METHOD5) \
    /* Requests global context id of the inspected tab. */ \
    METHOD0(getContextId) \
    \
    /* Request v8 to process all debug commands in the queue. */ \
    METHOD0(processDebugCommands)

DEFINE_RPC_CLASS(DebuggerAgent, DEBUGGER_AGENT_STRUCT)

#define DEBUGGER_AGENT_DELEGATE_STRUCT(METHOD0, METHOD1, METHOD2, METHOD3, METHOD4, METHOD5) \
    METHOD1(debuggerOutput, String /* output text */) \
    \
    /* Pushes debugger context id into the client. */ \
    METHOD1(setContextId, int /* context id */)

DEFINE_RPC_CLASS(DebuggerAgentDelegate, DEBUGGER_AGENT_DELEGATE_STRUCT)

} // namespace WebKit

#endif
