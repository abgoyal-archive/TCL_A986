

#include "config.h"
#include "DebuggerCallFrame.h"

#include "JSFunction.h"
#include "CodeBlock.h"
#include "Interpreter.h"
#include "Parser.h"

namespace JSC {

const UString* DebuggerCallFrame::functionName() const
{
    if (!m_callFrame->codeBlock())
        return 0;

    JSFunction* function = asFunction(m_callFrame->callee());
    if (!function)
        return 0;
    return &function->name(m_callFrame);
}
    
UString DebuggerCallFrame::calculatedFunctionName() const
{
    if (!m_callFrame->codeBlock())
        return UString();
    
    JSFunction* function = asFunction(m_callFrame->callee());
    if (!function)
        return UString();
    return function->calculatedDisplayName(m_callFrame);
}

DebuggerCallFrame::Type DebuggerCallFrame::type() const
{
    if (m_callFrame->callee())
        return FunctionType;

    return ProgramType;
}

JSObject* DebuggerCallFrame::thisObject() const
{
    if (!m_callFrame->codeBlock())
        return 0;

    return asObject(m_callFrame->thisValue());
}

JSValue DebuggerCallFrame::evaluate(const UString& script, JSValue& exception) const
{
    if (!m_callFrame->codeBlock())
        return JSValue();

    RefPtr<EvalExecutable> eval = EvalExecutable::create(m_callFrame, makeSource(script));
    JSObject* error = eval->compile(m_callFrame, m_callFrame->scopeChain());
    if (error)
        return error;

    return m_callFrame->scopeChain()->globalData->interpreter->execute(eval.get(), m_callFrame, thisObject(), m_callFrame->scopeChain(), &exception);
}

} // namespace JSC
