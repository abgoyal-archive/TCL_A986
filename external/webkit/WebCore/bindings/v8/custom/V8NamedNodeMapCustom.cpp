

#include "config.h"
#include "V8NamedNodeMap.h"

#include "NamedNodeMap.h"
#include "V8Binding.h"
#include "V8Element.h"
#include "V8Node.h"
#include "V8Proxy.h"

#include <wtf/RefPtr.h>

namespace WebCore {

v8::Handle<v8::Value> V8NamedNodeMap::indexedPropertyGetter(uint32_t index, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.NamedNodeMap.IndexedPropertyGetter");
    NamedNodeMap* imp = V8NamedNodeMap::toNative(info.Holder());
    RefPtr<Node> result = imp->item(index);
    if (!result)
        return notHandledByInterceptor();

    return toV8(result.release());
}

v8::Handle<v8::Value> V8NamedNodeMap::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.NamedNodeMap.NamedPropertyGetter");
    // Search the prototype chain first.
    v8::Handle<v8::Value> value = info.Holder()->GetRealNamedPropertyInPrototypeChain(name);
    if (!value.IsEmpty())
        return value;

    // Then look for IDL defined properties on the object itself.
    if (info.Holder()->HasRealNamedCallbackProperty(name))
        return notHandledByInterceptor();

    // Finally, search the DOM.
    NamedNodeMap* imp = V8NamedNodeMap::toNative(info.Holder());
    RefPtr<Node> result = imp->getNamedItem(toWebCoreString(name));
    if (!result)
        return notHandledByInterceptor();

    return toV8(result.release());
}

v8::Handle<v8::Value> toV8(NamedNodeMap* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8NamedNodeMap::wrap(impl);
    // Add a hidden reference from named node map to its owner node.
    Element* element = impl->element();
    if (!wrapper.IsEmpty() && element) {
        v8::Handle<v8::Value> owner = toV8(element);
        wrapper->SetInternalField(V8NamedNodeMap::ownerNodeIndex, owner);
    }
    return wrapper;
}

} // namespace WebCore
