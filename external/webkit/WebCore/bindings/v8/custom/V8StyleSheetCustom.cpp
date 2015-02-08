

#include "config.h"
#include "V8StyleSheet.h"

#include "V8CSSStyleSheet.h"
#include "V8Node.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(StyleSheet* impl)
{
    if (!impl)
        return v8::Null();
    if (impl->isCSSStyleSheet())
        return toV8(static_cast<CSSStyleSheet*>(impl));
    v8::Handle<v8::Object> wrapper = V8StyleSheet::wrap(impl);
    // Add a hidden reference from stylesheet object to its owner node.
    Node* ownerNode = impl->ownerNode();
    if (ownerNode && !wrapper.IsEmpty()) {
        v8::Handle<v8::Object> owner = v8::Handle<v8::Object>::Cast(toV8(ownerNode));
        wrapper->SetInternalField(V8StyleSheet::ownerNodeIndex, owner);
    }
    return wrapper;
}

} // namespace WebCore
