

#include "config.h"
#include "ProfilerAgentImpl.h"

#include <v8.h>

namespace WebKit {

void ProfilerAgentImpl::getActiveProfilerModules()
{
    m_delegate->didGetActiveProfilerModules(v8::V8::GetActiveProfilerModules());
}

void ProfilerAgentImpl::getLogLines(int position)
{
    static char buffer[65536];
    const int readSize = v8::V8::GetLogLines(position, buffer, sizeof(buffer) - 1);
    buffer[readSize] = '\0';
    position += readSize;
    m_delegate->didGetLogLines(position, buffer);
}

} // namespace WebKit
