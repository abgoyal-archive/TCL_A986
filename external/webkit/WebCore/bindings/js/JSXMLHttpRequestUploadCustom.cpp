

#include "config.h"
#include "JSXMLHttpRequestUpload.h"

#include "DOMWindow.h"
#include "Document.h"
#include "Event.h"
#include "Frame.h"
#include "JSDOMWindowCustom.h"
#include "JSEvent.h"
#include "JSEventListener.h"
#include "XMLHttpRequest.h"
#include "XMLHttpRequestUpload.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

void JSXMLHttpRequestUpload::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    if (XMLHttpRequest* xmlHttpRequest = m_impl->associatedXMLHttpRequest())
        markDOMObjectWrapper(markStack, *Heap::heap(this)->globalData(), xmlHttpRequest);

    m_impl->markJSEventListeners(markStack);
}

JSValue JSXMLHttpRequestUpload::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSXMLHttpRequestUpload::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

} // namespace WebCore
