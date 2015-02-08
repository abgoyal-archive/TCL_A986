

#ifndef V8HiddenPropertyName_h
#define V8HiddenPropertyName_h

#include <v8.h>

namespace WebCore {

#define V8_HIDDEN_PROPERTIES(V) \
    V(objectPrototype) \
    V(listener) \
    V(attributeListener) \
    V(scriptState) \
    V(sleepFunction) \
    V(toStringString)

    class V8HiddenPropertyName {
    public:
#define V8_DECLARE_PROPERTY(name) static v8::Handle<v8::String> name();
        V8_HIDDEN_PROPERTIES(V8_DECLARE_PROPERTY);
#undef V8_DECLARE_PROPERTY

    private:
        static v8::Persistent<v8::String>* createString(const char* key);
    };

}

#endif // V8HiddenPropertyName_h
