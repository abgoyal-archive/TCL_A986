

#include "config.h"
#include "ScriptValue.h"

#include "ScriptScope.h"
#include "SerializedScriptValue.h"
#include "V8Binding.h"

namespace WebCore {

PassRefPtr<SerializedScriptValue> ScriptValue::serialize(ScriptState* scriptState)
{
    ScriptScope scope(scriptState);
    return SerializedScriptValue::create(v8Value());
}

ScriptValue ScriptValue::deserialize(ScriptState* scriptState, SerializedScriptValue* value)
{
    ScriptScope scope(scriptState);
    return ScriptValue(value->deserialize());
}

bool ScriptValue::getString(String& result) const
{
    if (m_value.IsEmpty())
        return false;

    if (!m_value->IsString())
        return false;

    result = toWebCoreString(m_value);
    return true;
}

String ScriptValue::toString(ScriptState*) const
{
    return toWebCoreStringWithNullCheck(m_value);
}

} // namespace WebCore
