

#ifndef JSPluginElementFunctions_h
#define JSPluginElementFunctions_h

#include "JSDOMBinding.h"

namespace WebCore {

    class HTMLElement;
    class JSHTMLElement;
    class Node;

    // Runtime object support code for JSHTMLAppletElement, JSHTMLEmbedElement and JSHTMLObjectElement.

    JSC::JSValue runtimeObjectGetter(JSC::ExecState*, const JSC::Identifier&, const JSC::PropertySlot&);
    JSC::JSValue runtimeObjectPropertyGetter(JSC::ExecState*, const JSC::Identifier&, const JSC::PropertySlot&);
    bool runtimeObjectCustomGetOwnPropertySlot(JSC::ExecState*, const JSC::Identifier&, JSC::PropertySlot&, JSHTMLElement*);
    bool runtimeObjectCustomGetOwnPropertyDescriptor(JSC::ExecState*, const JSC::Identifier&, JSC::PropertyDescriptor&, JSHTMLElement*);
    bool runtimeObjectCustomPut(JSC::ExecState*, const JSC::Identifier&, JSC::JSValue, HTMLElement*, JSC::PutPropertySlot&);
    JSC::CallType runtimeObjectGetCallData(HTMLElement*, JSC::CallData&);

} // namespace WebCore

#endif // JSPluginElementFunctions_h
