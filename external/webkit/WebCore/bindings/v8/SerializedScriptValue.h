

#ifndef SerializedScriptValue_h
#define SerializedScriptValue_h

#include "ScriptValue.h"
#include "V8Binding.h"
#include <v8.h>
#include <wtf/RefCounted.h>

namespace WebCore {

class SerializedScriptValue : public RefCounted<SerializedScriptValue> {
public:
    // Creates a serialized representation of the given V8 value.
    static PassRefPtr<SerializedScriptValue> create(v8::Handle<v8::Value> value)
    {
        return adoptRef(new SerializedScriptValue(value));
    }

    // Creates a serialized value with the given data obtained from a
    // prior call to toWireString().
    static PassRefPtr<SerializedScriptValue> createFromWire(String data)
    {
        return adoptRef(new SerializedScriptValue(data, WireData));
    }

    // Creates a serialized representation of WebCore string.
    static PassRefPtr<SerializedScriptValue> create(String data)
    {
        return adoptRef(new SerializedScriptValue(data, StringValue));
    }

    // Creates an empty serialized value.
    static PassRefPtr<SerializedScriptValue> create()
    {
        return adoptRef(new SerializedScriptValue());
    }

    PassRefPtr<SerializedScriptValue> release()
    {
        RefPtr<SerializedScriptValue> result = adoptRef(new SerializedScriptValue(m_data, WireData));
        m_data = String();
        return result.release();
    }

    String toWireString() const { return m_data; }

    // Deserializes the value (in the current context). Returns an
    // empty handle in case of failure.
    v8::Local<v8::Value> deserialize();

private:
    enum StringDataMode {
        StringValue,
        WireData
    };

    SerializedScriptValue() { }

    explicit SerializedScriptValue(v8::Handle<v8::Value>);

    SerializedScriptValue(String data, StringDataMode mode);

    String m_data;
};

} // namespace WebCore

#endif // SerializedScriptValue_h
