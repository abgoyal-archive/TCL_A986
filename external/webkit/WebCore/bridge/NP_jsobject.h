

#ifndef NP_JSOBJECT_H
#define NP_JSOBJECT_H

#if ENABLE(NETSCAPE_PLUGIN_API)

#include "npruntime_internal.h"
#include <wtf/Forward.h>

namespace JSC {
    class JSObject;
    namespace Bindings {
        class RootObject;
    }
}

extern NPClass* NPScriptObjectClass;

struct JavaScriptObject
{
    NPObject object;
    JSC::JSObject* imp;
    JSC::Bindings::RootObject* rootObject;
};

NPObject* _NPN_CreateScriptObject(NPP npp, JSC::JSObject*, PassRefPtr<JSC::Bindings::RootObject> rootObject);
NPObject* _NPN_CreateNoScriptObject(void);

#endif // ENABLE(NETSCAPE_PLUGIN_API)

#endif
