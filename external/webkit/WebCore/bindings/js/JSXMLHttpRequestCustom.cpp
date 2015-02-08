

#include "config.h"
#include "JSXMLHttpRequest.h"

#include "Blob.h"
#include "DOMWindow.h"
#include "Document.h"
#include "Event.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "HTMLDocument.h"
#include "JSBlob.h"
#include "JSDOMWindowCustom.h"
#include "JSDocument.h"
#include "JSEvent.h"
#include "JSEventListener.h"
#include "XMLHttpRequest.h"
#include <runtime/Error.h>
#include <interpreter/Interpreter.h>

using namespace JSC;

namespace WebCore {

void JSXMLHttpRequest::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    if (XMLHttpRequestUpload* upload = m_impl->optionalUpload())
        markDOMObjectWrapper(markStack, *Heap::heap(this)->globalData(), upload);

    m_impl->markJSEventListeners(markStack);
}

// Custom functions
JSValue JSXMLHttpRequest::open(ExecState* exec, const ArgList& args)
{
    if (args.size() < 2)
        return throwError(exec, SyntaxError, "Not enough arguments");

    const KURL& url = impl()->scriptExecutionContext()->completeURL(args.at(1).toString(exec));
    String method = args.at(0).toString(exec);
    bool async = true;
    if (args.size() >= 3)
        async = args.at(2).toBoolean(exec);

    ExceptionCode ec = 0;
    if (args.size() >= 4 && !args.at(3).isUndefined()) {
        String user = valueToStringWithNullCheck(exec, args.at(3));

        if (args.size() >= 5 && !args.at(4).isUndefined()) {
            String password = valueToStringWithNullCheck(exec, args.at(4));
            impl()->open(method, url, async, user, password, ec);
        } else
            impl()->open(method, url, async, user, ec);
    } else
        impl()->open(method, url, async, ec);

    setDOMException(exec, ec);
    return jsUndefined();
}

JSValue JSXMLHttpRequest::setRequestHeader(ExecState* exec, const ArgList& args)
{
    if (args.size() < 2)
        return throwError(exec, SyntaxError, "Not enough arguments");

    ExceptionCode ec = 0;
    impl()->setRequestHeader(args.at(0).toString(exec), args.at(1).toString(exec), ec);
    setDOMException(exec, ec);
    return jsUndefined();
}

JSValue JSXMLHttpRequest::send(ExecState* exec, const ArgList& args)
{
    ExceptionCode ec = 0;
    if (args.isEmpty())
        impl()->send(ec);
    else {
        JSValue val = args.at(0);
        if (val.isUndefinedOrNull())
            impl()->send(ec);
        else if (val.inherits(&JSDocument::s_info))
            impl()->send(toDocument(val), ec);
        else if (val.inherits(&JSBlob::s_info))
            impl()->send(toBlob(val), ec);
        else
            impl()->send(val.toString(exec), ec);
    }

    int signedLineNumber;
    intptr_t sourceID;
    UString sourceURL;
    JSValue function;
    exec->interpreter()->retrieveLastCaller(exec, signedLineNumber, sourceID, sourceURL, function);
    impl()->setLastSendLineNumber(signedLineNumber >= 0 ? signedLineNumber : 0);
    impl()->setLastSendURL(sourceURL);

    setDOMException(exec, ec);
    return jsUndefined();
}

JSValue JSXMLHttpRequest::getResponseHeader(ExecState* exec, const ArgList& args)
{
    if (args.size() < 1)
        return throwError(exec, SyntaxError, "Not enough arguments");

    ExceptionCode ec = 0;
    JSValue header = jsStringOrNull(exec, impl()->getResponseHeader(args.at(0).toString(exec), ec));
    setDOMException(exec, ec);
    return header;
}

JSValue JSXMLHttpRequest::overrideMimeType(ExecState* exec, const ArgList& args)
{
    if (args.size() < 1)
        return throwError(exec, SyntaxError, "Not enough arguments");

    impl()->overrideMimeType(args.at(0).toString(exec));
    return jsUndefined();
}

JSValue JSXMLHttpRequest::addEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->addEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSXMLHttpRequest::removeEventListener(ExecState* exec, const ArgList& args)
{
    JSValue listener = args.at(1);
    if (!listener.isObject())
        return jsUndefined();

    impl()->removeEventListener(args.at(0).toString(exec), JSEventListener::create(asObject(listener), this, false, currentWorld(exec)).get(), args.at(2).toBoolean(exec));
    return jsUndefined();
}

JSValue JSXMLHttpRequest::responseText(ExecState* exec) const
{
    return jsOwnedStringOrNull(exec, impl()->responseText());
}

} // namespace WebCore
