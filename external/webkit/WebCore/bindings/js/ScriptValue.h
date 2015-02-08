

#ifndef ScriptValue_h
#define ScriptValue_h

#include "PlatformString.h"
#include "ScriptState.h"
#include <runtime/Protect.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

class String;
class SerializedScriptValue;

class ScriptValue {
public:
    ScriptValue(JSC::JSValue value = JSC::JSValue()) : m_value(value) {}
    virtual ~ScriptValue() {}

    JSC::JSValue jsValue() const { return m_value.get(); }
    bool getString(ScriptState*, String& result) const;
    String toString(ScriptState* scriptState) const { return m_value.get().toString(scriptState); }
    bool isEqual(ScriptState*, const ScriptValue&) const;
    bool isNull() const;
    bool isUndefined() const;
    bool isObject() const;
    bool hasNoValue() const { return m_value == JSC::JSValue(); }

    PassRefPtr<SerializedScriptValue> serialize(ScriptState*);
    static ScriptValue deserialize(ScriptState*, SerializedScriptValue*);

private:
    JSC::ProtectedJSValue m_value;
};

} // namespace WebCore

#endif // ScriptValue_h
