

#include "config.h"
#include "ScriptCallStack.h"

#include "ScriptController.h"

#include <v8.h>

#include "V8Binding.h"
#include "V8Proxy.h"

namespace WebCore {

ScriptCallStack* ScriptCallStack::create(const v8::Arguments& arguments, unsigned skipArgumentCount) {
    String sourceName;
    int sourceLineNumber;
    if (!V8Proxy::sourceName(sourceName)) {
        return 0;
    }
    if (!V8Proxy::sourceLineNumber(sourceLineNumber)) {
        return 0;
    }
    sourceLineNumber += 1;
    return new ScriptCallStack(arguments, skipArgumentCount, sourceName, sourceLineNumber);
}

ScriptCallStack::ScriptCallStack(const v8::Arguments& arguments, unsigned skipArgumentCount, String sourceName, int sourceLineNumber)
    : m_lastCaller(String(), sourceName, sourceLineNumber, arguments, skipArgumentCount)
    , m_scriptState(ScriptState::current())
{
}

ScriptCallStack::~ScriptCallStack()
{
}

const ScriptCallFrame& ScriptCallStack::at(unsigned index) const
{
    // Currently, only one ScriptCallFrame is supported. When we can get
    // a full stack trace from V8, we can do this right.
    ASSERT(index == 0);
    return m_lastCaller;
}

} // namespace WebCore
