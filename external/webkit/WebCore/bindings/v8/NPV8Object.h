
#ifndef NPV8Object_h
#define NPV8Object_h

#include "V8Index.h"

#if PLATFORM(CHROMIUM)
// FIXME: Chromium uses a different npruntime.h, which is in
// the Chromium source repository under third_party/npapi/bindings.
// The Google-specific changes in that file should probably be
// moved into bridge/npruntime.h, guarded by an #if PlATFORM(CHROMIUM).
#include <bindings/npruntime.h>
#else
#include "npruntime.h" // Use WebCore version for Android and other ports.
#endif

#include <v8.h>

namespace WebCore {
    class DOMWindow;

    static const int npObjectInternalFieldCount = v8DefaultWrapperInternalFieldCount + 0;
}

extern NPClass* npScriptObjectClass;

// A V8NPObject is a NPObject which carries additional V8-specific information. It is allocated and deallocated by
// AllocV8NPObject() and FreeV8NPObject() methods.
struct V8NPObject {
    NPObject object;
    v8::Persistent<v8::Object> v8Object;
    WebCore::DOMWindow* rootObject;
};

struct PrivateIdentifier {
    union {
        const NPUTF8* string;
        int32_t number;
    } value;
    bool isString;
};

NPObject* npCreateV8ScriptObject(NPP, v8::Handle<v8::Object>, WebCore::DOMWindow*);

NPObject* v8ObjectToNPObject(v8::Handle<v8::Object>);

#endif // NPV8Object_h
