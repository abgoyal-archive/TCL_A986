

#include "config.h"
#include "ScriptCallStack.h"

#include <interpreter/CallFrame.h>
#include <interpreter/Interpreter.h>
#include <runtime/InternalFunction.h>
#include <runtime/JSValue.h>
#include <runtime/UString.h>
#include <runtime/JSGlobalData.h>

using namespace JSC;

namespace WebCore {

ScriptCallStack::ScriptCallStack(ExecState* exec, const ArgList& args, unsigned skipArgumentCount)
    : m_initialized(false)
    , m_exec(exec)
    , m_caller(0)
{
    int signedLineNumber;
    intptr_t sourceID;
    UString urlString;
    JSValue function;

    exec->interpreter()->retrieveLastCaller(exec, signedLineNumber, sourceID, urlString, function);

    unsigned lineNumber = signedLineNumber >= 0 ? signedLineNumber : 0;

    if (function) {
        m_caller = asInternalFunction(function);
        m_frames.append(ScriptCallFrame(m_caller->name(m_exec), urlString, lineNumber, args, skipArgumentCount));
    } else {
        // Caller is unknown, but we should still add the frame, because
        // something called us, and gave us arguments.
        m_frames.append(ScriptCallFrame(UString(), urlString, lineNumber, args, skipArgumentCount));
    }
}

ScriptCallStack::~ScriptCallStack()
{
}

const ScriptCallFrame &ScriptCallStack::at(unsigned index)
{
    // First frame is pre-populated in constructor, so don't trigger
    // initialization unless looking beyond the first frame.
    if (index > 0)
        initialize();
    ASSERT(m_frames.size() > index);
    return m_frames[index];
}

unsigned ScriptCallStack::size()
{
    initialize();
    return m_frames.size();
}

void ScriptCallStack::initialize()
{
    if (!m_caller || m_initialized)
        return;

    JSValue func = m_exec->interpreter()->retrieveCaller(m_exec, m_caller);
    while (!func.isNull()) {
        InternalFunction* internalFunction = asInternalFunction(func);
        ArgList emptyArgList;
        m_frames.append(ScriptCallFrame(internalFunction->name(m_exec), UString(), 0, emptyArgList, 0));
        func = m_exec->interpreter()->retrieveCaller(m_exec, internalFunction);
    }
    m_initialized = true;
}

} // namespace WebCore
