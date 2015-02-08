

#include "config.h"
#include "JavaScriptCallFrame.h"
#include "JSDOMBinding.h"

#if ENABLE(JAVASCRIPT_DEBUGGER) && USE(JSC)

#include "PlatformString.h"
#include <debugger/DebuggerCallFrame.h>
#include <runtime/JSGlobalObject.h>
#include <runtime/Completion.h>
#include <runtime/JSLock.h>
#include <runtime/JSObject.h>
#include <runtime/JSValue.h>

using namespace JSC;

namespace WebCore {
    
JavaScriptCallFrame::JavaScriptCallFrame(const DebuggerCallFrame& debuggerCallFrame, PassRefPtr<JavaScriptCallFrame> caller, intptr_t sourceID, int line)
    : m_debuggerCallFrame(debuggerCallFrame)
    , m_caller(caller)
    , m_sourceID(sourceID)
    , m_line(line)
    , m_isValid(true)
{
}

JavaScriptCallFrame* JavaScriptCallFrame::caller()
{
    return m_caller.get();
}

const JSC::ScopeChainNode* JavaScriptCallFrame::scopeChain() const
{
    ASSERT(m_isValid);
    if (!m_isValid)
        return 0;
    return m_debuggerCallFrame.scopeChain();
}

JSC::JSGlobalObject* JavaScriptCallFrame::dynamicGlobalObject() const
{
    ASSERT(m_isValid);
    if (!m_isValid)
        return 0;
    return m_debuggerCallFrame.dynamicGlobalObject();
}

String JavaScriptCallFrame::functionName() const
{
    ASSERT(m_isValid);
    if (!m_isValid)
        return String();
    UString functionName = m_debuggerCallFrame.calculatedFunctionName();
    if (functionName.isEmpty())
        return String();
    return functionName;
}

DebuggerCallFrame::Type JavaScriptCallFrame::type() const
{
    ASSERT(m_isValid);
    if (!m_isValid)
        return DebuggerCallFrame::ProgramType;
    return m_debuggerCallFrame.type();
}

JSObject* JavaScriptCallFrame::thisObject() const
{
    ASSERT(m_isValid);
    if (!m_isValid)
        return 0;
    return m_debuggerCallFrame.thisObject();
}

// Evaluate some JavaScript code in the scope of this frame.
JSValue JavaScriptCallFrame::evaluate(const UString& script, JSValue& exception) const
{
    ASSERT(m_isValid);
    if (!m_isValid)
        return jsNull();

    JSLock lock(SilenceAssertionsOnly);
    return m_debuggerCallFrame.evaluate(script, exception);
}

} // namespace WebCore

#endif // ENABLE(JAVASCRIPT_DEBUGGER)
