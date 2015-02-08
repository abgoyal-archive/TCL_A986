

#include "config.h"
#include "V8Helpers.h"

#include "DOMWindow.h"
#include "NPV8Object.h"
#include "V8Index.h"
#include "V8Proxy.h"

namespace WebCore {

void wrapNPObject(v8::Handle<v8::Object> object, NPObject* npObject)
{
    V8DOMWrapper::setDOMWrapper(object, V8ClassIndex::NPOBJECT, npObject);
}

v8::Local<v8::Context> toV8Context(NPP npp, NPObject* npObject)
{
    V8NPObject* object = reinterpret_cast<V8NPObject*>(npObject);
    return V8Proxy::mainWorldContext(object->rootObject->frame());
}

V8Proxy* toV8Proxy(NPObject* npObject)
{
    V8NPObject* object = reinterpret_cast<V8NPObject*>(npObject);
    Frame* frame = object->rootObject->frame();
    return V8Proxy::retrieve(frame);
}

} // namespace WebCore
