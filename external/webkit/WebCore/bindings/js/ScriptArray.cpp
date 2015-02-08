

#include "config.h"
#include "ScriptArray.h"

#include <runtime/JSLock.h>

using namespace JSC;

namespace WebCore {

ScriptArray::ScriptArray(ScriptState* scriptState, JSArray* object)
    : ScriptObject(scriptState, object)
{
}

static bool handleException(ScriptState* scriptState)
{
    if (!scriptState->hadException())
        return true;

    reportException(scriptState, scriptState->exception());
    return false;
}

bool ScriptArray::set(unsigned index, const ScriptObject& value)
{
    if (value.scriptState() != m_scriptState) {
        ASSERT_NOT_REACHED();
        return false;
    }
    JSLock lock(SilenceAssertionsOnly);
    jsArray()->put(m_scriptState, index, value.jsObject());
    return handleException(m_scriptState);
}

bool ScriptArray::set(unsigned index, const String& value)
{
    JSLock lock(SilenceAssertionsOnly);
    jsArray()->put(m_scriptState, index, jsString(m_scriptState, value));
    return handleException(m_scriptState);
}

bool ScriptArray::set(unsigned index, double value)
{
    JSLock lock(SilenceAssertionsOnly);
    jsArray()->put(m_scriptState, index, jsNumber(m_scriptState, value));
    return handleException(m_scriptState);
}

bool ScriptArray::set(unsigned index, long long value)
{
    JSLock lock(SilenceAssertionsOnly);
    jsArray()->put(m_scriptState, index, jsNumber(m_scriptState, value));
    return handleException(m_scriptState);
}

bool ScriptArray::set(unsigned index, int value)
{
    JSLock lock(SilenceAssertionsOnly);
    jsArray()->put(m_scriptState, index, jsNumber(m_scriptState, value));
    return handleException(m_scriptState);
}

bool ScriptArray::set(unsigned index, bool value)
{
    JSLock lock(SilenceAssertionsOnly);
    jsArray()->put(m_scriptState, index, jsBoolean(value));
    return handleException(m_scriptState);
}

unsigned ScriptArray::length()
{
    JSLock lock(SilenceAssertionsOnly);
    return jsArray()->length();
}

ScriptArray ScriptArray::createNew(ScriptState* scriptState)
{
    JSLock lock(SilenceAssertionsOnly);
    return ScriptArray(scriptState, constructEmptyArray(scriptState));
}

} // namespace WebCore
