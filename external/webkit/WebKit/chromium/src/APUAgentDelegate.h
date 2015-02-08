

#ifndef APUAgentDelegate_h
#define APUAgentDelegate_h

#include "DevToolsRPC.h"

namespace WebKit {

#define APU_AGENT_DELEGATE_STRUCT(METHOD0, METHOD1, METHOD2, METHOD3, MEHTOD4, METHOD5) \
    /* Sends a json object to apu. */ \
    METHOD1(dispatchToApu, String /* data */)

DEFINE_RPC_CLASS(ApuAgentDelegate, APU_AGENT_DELEGATE_STRUCT)

} // namespace WebKit

#endif
