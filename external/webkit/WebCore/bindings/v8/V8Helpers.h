

#ifndef V8Helpers_h
#define V8Helpers_h

#include "npruntime.h"
#include <v8.h>

namespace WebCore {
    class V8Proxy;

    // Associates an NPObject with a V8 object.
    void wrapNPObject(v8::Handle<v8::Object>, NPObject*);

    v8::Local<v8::Context> toV8Context(NPP, NPObject*);

    V8Proxy* toV8Proxy(NPObject*);

} // namespace WebCore

#endif // V8Helpers_h
