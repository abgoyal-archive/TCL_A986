

#include "config.h"

#if ENABLE(INDEXED_DATABASE)
#include "V8IDBRequest.h"

#include "SerializedScriptValue.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8IDBRequest::resultAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    IDBRequest* request = V8IDBRequest::toNative(info.Holder());
    SerializedScriptValue* result = request->result();
    if (!result)
        return v8::Null();

    return result->deserialize();
}

} // namespace WebCore

#endif
