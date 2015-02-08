

#include "config.h"
#include "V8CSSStyleSheet.h"

#include "V8Node.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(CSSStyleSheet* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8CSSStyleSheet::wrap(impl);
    // Add a hidden reference from stylesheet object to its owner node.
    Node* ownerNode = impl->ownerNode();
    if (ownerNode && !wrapper.IsEmpty())
        wrapper->SetInternalField(V8CSSStyleSheet::ownerNodeIndex, toV8(ownerNode));
    return wrapper;
}

} // namespace WebCore
