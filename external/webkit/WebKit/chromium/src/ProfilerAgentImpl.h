

#ifndef ProfilerAgentImpl_h
#define ProfilerAgentImpl_h

#include "ProfilerAgent.h"

namespace WebKit {

class ProfilerAgentImpl : public ProfilerAgent {
public:
    ProfilerAgentImpl(ProfilerAgentDelegate* delegate) : m_delegate(delegate) { }
    virtual ~ProfilerAgentImpl() { }

    // ProfilerAgent implementation.

    // This method is called on IO thread.
    virtual void getActiveProfilerModules();

    // This method is called on IO thread.
    virtual void getLogLines(int position);

private:
    ProfilerAgentDelegate* m_delegate;
};

} // namespace WebKit

#endif
