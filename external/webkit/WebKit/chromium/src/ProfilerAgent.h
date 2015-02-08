

#ifndef ProfilerAgent_h
#define ProfilerAgent_h

#include "DevToolsRPC.h"

namespace WebKit {

// Profiler agent provides API for retrieving profiler data.
// These methods are handled on the IO thread, so profiler can
// operate while a script on a page performs heavy work.
#define PROFILER_AGENT_STRUCT(METHOD0, METHOD1, METHOD2, METHOD3, METHOD4, METHOD5) \
    /* Requests current profiler state. */ \
    METHOD0(getActiveProfilerModules) \
    \
    /* Retrieves portion of profiler log. */ \
    METHOD1(getLogLines, int /* position */)

DEFINE_RPC_CLASS(ProfilerAgent, PROFILER_AGENT_STRUCT)

#define PROFILER_AGENT_DELEGATE_STRUCT(METHOD0, METHOD1, METHOD2, METHOD3, METHOD4, METHOD5) \
    /* Response to getActiveProfilerModules. */ \
    METHOD1(didGetActiveProfilerModules, int /* flags */) \
    \
    /* Response to getLogLines. */ \
    METHOD2(didGetLogLines, int /* position */, String /* log */)

DEFINE_RPC_CLASS(ProfilerAgentDelegate, PROFILER_AGENT_DELEGATE_STRUCT)

} // namespace WebKit

#endif
