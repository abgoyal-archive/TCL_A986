

#include "config.h"
#include "V8HiddenPropertyName.h"

namespace WebCore {

#define V8_AS_STRING(x) V8_AS_STRING_IMPL(x)
#define V8_AS_STRING_IMPL(x) #x

#define V8_DEFINE_PROPERTY(name) \
v8::Handle<v8::String> V8HiddenPropertyName::name() \
{ \
    static v8::Persistent<v8::String>* string = createString("WebCore::V8HiddenPropertyName::" V8_AS_STRING(name)); \
    return *string; \
}

V8_HIDDEN_PROPERTIES(V8_DEFINE_PROPERTY);

v8::Persistent<v8::String>* V8HiddenPropertyName::createString(const char* key)
{
    v8::HandleScope scope;
    return new v8::Persistent<v8::String>(v8::Persistent<v8::String>::New(v8::String::NewSymbol(key)));
}

}  // namespace WebCore
