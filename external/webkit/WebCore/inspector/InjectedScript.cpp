

#include "config.h"
#include "InjectedScript.h"

#if ENABLE(INSPECTOR)

#include "PlatformString.h"
#include "SerializedScriptValue.h"
#include "ScriptFunctionCall.h"

namespace WebCore {

InjectedScript::InjectedScript(ScriptObject injectedScriptObject)
    : m_injectedScriptObject(injectedScriptObject)
{
}

void InjectedScript::dispatch(long callId, const String& methodName, const String& arguments, bool async, RefPtr<SerializedScriptValue>* result, bool* hadException) 
{
    ASSERT(!hasNoValue());
    ScriptFunctionCall function(m_injectedScriptObject, "dispatch");
    function.appendArgument(methodName);
    function.appendArgument(arguments);
    if (async)
        function.appendArgument(callId);
    *hadException = false;
    ScriptValue resultValue = function.call(*hadException);
    if (!*hadException)
        *result = resultValue.serialize(m_injectedScriptObject.scriptState());
}

#if ENABLE(JAVASCRIPT_DEBUGGER)
PassRefPtr<SerializedScriptValue> InjectedScript::callFrames()
{
    ASSERT(!hasNoValue());
    ScriptFunctionCall function(m_injectedScriptObject, "callFrames");
    ScriptValue callFramesValue = function.call();
    return callFramesValue.serialize(m_injectedScriptObject.scriptState());
}
#endif

PassRefPtr<SerializedScriptValue> InjectedScript::wrapForConsole(ScriptValue value)
{
    ASSERT(!hasNoValue());
    ScriptFunctionCall wrapFunction(m_injectedScriptObject, "wrapObject");
    wrapFunction.appendArgument(value);
    wrapFunction.appendArgument("console");
    ScriptValue r = wrapFunction.call();
    return r.serialize(m_injectedScriptObject.scriptState());
}

void InjectedScript::releaseWrapperObjectGroup(const String& objectGroup)
{
    ASSERT(!hasNoValue());
    ScriptFunctionCall releaseFunction(m_injectedScriptObject, "releaseWrapperObjectGroup");
    releaseFunction.appendArgument(objectGroup);
    releaseFunction.call();
}

} // namespace WebCore

#endif // ENABLE(INSPECTOR)
