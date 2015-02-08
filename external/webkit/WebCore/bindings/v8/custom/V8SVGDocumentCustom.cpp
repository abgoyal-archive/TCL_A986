

#include "config.h"
#include "V8SVGDocument.h"

#include "V8IsolatedContext.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(SVGDocument* impl, bool forceNewObject)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8SVGDocument::wrap(impl, forceNewObject);
    if (wrapper.IsEmpty())
        return wrapper;
    if (!V8IsolatedContext::getEntered()) {
        if (V8Proxy* proxy = V8Proxy::retrieve(impl->frame()))
            proxy->windowShell()->updateDocumentWrapper(wrapper);
    }
    return wrapper;
}

} // namespace WebCore
