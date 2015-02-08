

#include "config.h"
#include "V8Element.h"

#include "Attr.h"
#include "CSSHelper.h"
#include "Document.h"
#include "Element.h"
#include "ExceptionCode.h"
#include "HTMLFrameElementBase.h"
#include "HTMLNames.h"
#include "Node.h"

#include "V8Attr.h"
#include "V8Binding.h"
#include "V8BindingState.h"
#include "V8HTMLElement.h"
#include "V8Proxy.h"
#include "V8SVGElement.h"

#include <wtf/RefPtr.h>

namespace WebCore {

v8::Handle<v8::Value> V8Element::setAttributeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Element.setAttribute()");
    Element* element = V8Element::toNative(args.Holder());
    String name = toWebCoreString(args[0]);
    String value = toWebCoreString(args[1]);

    ExceptionCode ec = 0;
    V8BindingElement::setAttribute(V8BindingState::Only(), element, name, value, ec);
    if (ec)
        return throwError(ec);

    return v8::Undefined();
}

v8::Handle<v8::Value> V8Element::setAttributeNodeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Element.setAttributeNode()");
    if (!V8Attr::HasInstance(args[0]))
        return throwError(TYPE_MISMATCH_ERR);

    Attr* newAttr = V8Attr::toNative(v8::Handle<v8::Object>::Cast(args[0]));
    Element* element = V8Element::toNative(args.Holder());

    ExceptionCode ec = 0;
    RefPtr<Attr> result = V8BindingElement::setAttributeNode(V8BindingState::Only(), element, newAttr, ec);
    if (ec)
        throwError(ec);

    return toV8(result.release());
}

v8::Handle<v8::Value> V8Element::setAttributeNSCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Element.setAttributeNS()");
    Element* element = V8Element::toNative(args.Holder());
    String namespaceURI = toWebCoreStringWithNullCheck(args[0]);
    String qualifiedName = toWebCoreString(args[1]);
    String value = toWebCoreString(args[2]);

    ExceptionCode ec = 0;
    V8BindingElement::setAttributeNS(V8BindingState::Only(), element, namespaceURI, qualifiedName, value, ec);
    if (ec)
        throwError(ec);

    return v8::Undefined();
}

v8::Handle<v8::Value> V8Element::setAttributeNodeNSCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Element.setAttributeNodeNS()");
    if (!V8Attr::HasInstance(args[0]))
        return throwError(TYPE_MISMATCH_ERR);

    Attr* newAttr = V8Attr::toNative(v8::Handle<v8::Object>::Cast(args[0]));
    Element* element = V8Element::toNative(args.Holder());

    ExceptionCode ec = 0;
    RefPtr<Attr> result = V8BindingElement::setAttributeNodeNS(V8BindingState::Only(), element, newAttr, ec);
    if (ec)
        throwError(ec);

    return toV8(result.release());
}

v8::Handle<v8::Value> toV8(Element* impl, bool forceNewObject)
{
    if (!impl)
        return v8::Null();
    if (impl->isHTMLElement())
        return toV8(static_cast<HTMLElement*>(impl), forceNewObject);
#if ENABLE(SVG)
    if (impl->isSVGElement())
        return toV8(static_cast<SVGElement*>(impl), forceNewObject);
#endif
    return V8Element::wrap(impl, forceNewObject);
}
} // namespace WebCore
