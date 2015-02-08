

#include "config.h"
#include "V8HTMLFrameSetElement.h"

#include "Document.h"
#include "Frame.h"
#include "HTMLCollection.h"
#include "HTMLFrameElement.h"
#include "HTMLFrameSetElement.h"
#include "HTMLNames.h"
#include "Node.h"

#include "V8Binding.h"
#include "V8DOMWindow.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8HTMLFrameSetElement::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.HTMLFrameSetElement.NamedPropertyGetter");
    HTMLFrameSetElement* imp = V8HTMLFrameSetElement::toNative(info.Holder());
    Node* frameNode = imp->children()->namedItem(v8StringToAtomicWebCoreString(name));
    if (frameNode && frameNode->hasTagName(HTMLNames::frameTag)) {
        Document* doc = static_cast<HTMLFrameElement*>(frameNode)->contentDocument();
        if (!doc)
            return v8::Undefined();
        if (Frame* frame = doc->frame())
            return toV8(frame->domWindow());
    }
    return notHandledByInterceptor();
}

} // namespace WebCore
