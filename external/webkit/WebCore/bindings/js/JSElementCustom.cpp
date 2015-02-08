


#include "config.h"
#include "JSElement.h"

#include "CSSHelper.h"
#include "Document.h"
#include "ExceptionCode.h"
#include "HTMLFrameElementBase.h"
#include "HTMLNames.h"
#include "JSAttr.h"
#include "JSHTMLElementWrapperFactory.h"
#include "JSNodeList.h"
#include "NodeList.h"

#if ENABLE(SVG)
#include "JSSVGElementWrapperFactory.h"
#include "SVGElement.h"
#endif

using namespace JSC;

namespace WebCore {

using namespace HTMLNames;

void JSElement::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    Element* element = impl();
    JSGlobalData& globalData = *Heap::heap(this)->globalData();

    markDOMObjectWrapper(markStack, globalData, element->attributeMap());
    if (element->isStyledElement())
        markDOMObjectWrapper(markStack, globalData, static_cast<StyledElement*>(element)->inlineStyleDecl());
}

static inline bool allowSettingSrcToJavascriptURL(ExecState* exec, Element* element, const String& name, const String& value)
{
    if ((element->hasTagName(iframeTag) || element->hasTagName(frameTag)) && equalIgnoringCase(name, "src") && protocolIsJavaScript(deprecatedParseURL(value))) {
        Document* contentDocument = static_cast<HTMLFrameElementBase*>(element)->contentDocument();
        if (contentDocument && !checkNodeSecurity(exec, contentDocument))
            return false;
    }
    return true;
}

JSValue JSElement::setAttribute(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    AtomicString name = args.at(0).toString(exec);
    AtomicString value = args.at(1).toString(exec);

    Element* imp = impl();
    if (!allowSettingSrcToJavascriptURL(exec, imp, name, value))
        return jsUndefined();

    imp->setAttribute(name, value, ec);
    setDOMException(exec, ec);
    return jsUndefined();
}

JSValue JSElement::setAttributeNode(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    Attr* newAttr = toAttr(args.at(0));
    if (!newAttr) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    Element* imp = impl();
    if (!allowSettingSrcToJavascriptURL(exec, imp, newAttr->name(), newAttr->value()))
        return jsUndefined();

    JSValue result = toJS(exec, globalObject(), WTF::getPtr(imp->setAttributeNode(newAttr, ec)));
    setDOMException(exec, ec);
    return result;
}

JSValue JSElement::setAttributeNS(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    AtomicString namespaceURI = valueToStringWithNullCheck(exec, args.at(0));
    AtomicString qualifiedName = args.at(1).toString(exec);
    AtomicString value = args.at(2).toString(exec);

    Element* imp = impl();
    if (!allowSettingSrcToJavascriptURL(exec, imp, qualifiedName, value))
        return jsUndefined();

    imp->setAttributeNS(namespaceURI, qualifiedName, value, ec);
    setDOMException(exec, ec);
    return jsUndefined();
}

JSValue JSElement::setAttributeNodeNS(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    Attr* newAttr = toAttr(args.at(0));
    if (!newAttr) {
        setDOMException(exec, TYPE_MISMATCH_ERR);
        return jsUndefined();
    }

    Element* imp = impl();
    if (!allowSettingSrcToJavascriptURL(exec, imp, newAttr->name(), newAttr->value()))
        return jsUndefined();

    JSValue result = toJS(exec, globalObject(), WTF::getPtr(imp->setAttributeNodeNS(newAttr, ec)));
    setDOMException(exec, ec);
    return result;
}

JSValue toJSNewlyCreated(ExecState* exec, JSDOMGlobalObject* globalObject, Element* element)
{
    if (!element)
        return jsNull();

    ASSERT(!getCachedDOMNodeWrapper(exec, element->document(), element));

    JSNode* wrapper;        
    if (element->isHTMLElement())
        wrapper = createJSHTMLWrapper(exec, globalObject, static_cast<HTMLElement*>(element));
#if ENABLE(SVG)
    else if (element->isSVGElement())
        wrapper = createJSSVGWrapper(exec, globalObject, static_cast<SVGElement*>(element));
#endif
    else
        wrapper = CREATE_DOM_NODE_WRAPPER(exec, globalObject, Element, element);

    return wrapper;    
}

} // namespace WebCore
