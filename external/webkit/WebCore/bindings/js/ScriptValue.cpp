

#include "config.h"
#include "ScriptValue.h"

#include "SerializedScriptValue.h"

#include <JavaScriptCore/APICast.h>
#include <JavaScriptCore/JSValueRef.h>

#include <runtime/JSLock.h>
#include <runtime/Protect.h>
#include <runtime/UString.h>

using namespace JSC;

namespace WebCore {

bool ScriptValue::getString(ScriptState* scriptState, String& result) const
{
    if (!m_value)
        return false;
    JSLock lock(SilenceAssertionsOnly);
    UString ustring;
    if (!m_value.get().getString(scriptState, ustring))
        return false;
    result = ustring;
    return true;
}

bool ScriptValue::isEqual(ScriptState* scriptState, const ScriptValue& anotherValue) const
{
    if (hasNoValue())
        return anotherValue.hasNoValue();

    return JSValueIsEqual(toRef(scriptState), toRef(scriptState, jsValue()), toRef(scriptState, anotherValue.jsValue()), 0);
}

bool ScriptValue::isNull() const
{
    if (!m_value)
        return false;
    return m_value.get().isNull();
}

bool ScriptValue::isUndefined() const
{
    if (!m_value)
        return false;
    return m_value.get().isUndefined();
}

bool ScriptValue::isObject() const
{
    if (!m_value)
        return false;
    return m_value.get().isObject();
}

PassRefPtr<SerializedScriptValue> ScriptValue::serialize(ScriptState* scriptState)
{
    return SerializedScriptValue::create(scriptState, jsValue());
}

ScriptValue ScriptValue::deserialize(ScriptState* scriptState, SerializedScriptValue* value)
{
    return ScriptValue(value->deserialize(scriptState, scriptState->lexicalGlobalObject()));
}

} // namespace WebCore
