

#ifndef JavaNPObjectV8_h
#define JavaNPObjectV8_h

#include "npruntime.h"
#include <wtf/RefPtr.h>


namespace JSC {

namespace Bindings {

class JavaInstance;

struct JavaNPObject {
    NPObject m_object;
    RefPtr<JavaInstance> m_instance;
};

// ANDROID-specific change. TODO: Upstream
NPObject* JavaInstanceToNPObject(PassRefPtr<JavaInstance>);
// END ANDROID-specific change
JavaInstance* ExtractJavaInstance(NPObject*);

bool JavaNPObjectHasMethod(NPObject*, NPIdentifier name);
bool JavaNPObjectInvoke(NPObject*, NPIdentifier methodName, const NPVariant* args, uint32_t argCount, NPVariant* result);
bool JavaNPObjectHasProperty(NPObject*, NPIdentifier name);
bool JavaNPObjectGetProperty(NPObject*, NPIdentifier name, NPVariant* result);

} // namespace Bindings

} // namespace JSC

#endif // JavaNPObjectV8_h
