

#include "config.h"
#include "V8XMLHttpRequest.h"

#include "Frame.h"
#include "V8Binding.h"
#include "V8Blob.h"
#include "V8Document.h"
#include "V8HTMLDocument.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"
#include "XMLHttpRequest.h"

namespace WebCore {

v8::Handle<v8::Value> V8XMLHttpRequest::responseTextAccessorGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.XMLHttpRequest.responsetext._get");
    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(info.Holder());
    return xmlHttpRequest->responseText().v8StringOrNull();
}

v8::Handle<v8::Value> V8XMLHttpRequest::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.addEventListener()");
    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(xmlHttpRequest, args[1], false, ListenerFindOrCreate);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        xmlHttpRequest->addEventListener(type, listener, useCapture);

        createHiddenDependency(args.Holder(), args[1], cacheIndex);
    }
    return v8::Undefined();
}

v8::Handle<v8::Value> V8XMLHttpRequest::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.removeEventListener()");
    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(xmlHttpRequest, args[1], false, ListenerFindOnly);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        xmlHttpRequest->removeEventListener(type, listener.get(), useCapture);

        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
    }

    return v8::Undefined();
}

v8::Handle<v8::Value> V8XMLHttpRequest::openCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.open()");
    // Four cases:
    // open(method, url)
    // open(method, url, async)
    // open(method, url, async, user)
    // open(method, url, async, user, passwd)

    if (args.Length() < 2)
        return throwError("Not enough arguments", V8Proxy::SyntaxError);

    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(args.Holder());

    String method = toWebCoreString(args[0]);
    String urlstring = toWebCoreString(args[1]);
    ScriptExecutionContext* context = getScriptExecutionContext();
    if (!context)
        return v8::Undefined();

    KURL url = context->completeURL(urlstring);

    bool async = (args.Length() < 3) ? true : args[2]->BooleanValue();

    ExceptionCode ec = 0;
    String user, passwd;
    if (args.Length() >= 4 && !args[3]->IsUndefined()) {
        user = toWebCoreStringWithNullCheck(args[3]);

        if (args.Length() >= 5 && !args[4]->IsUndefined()) {
            passwd = toWebCoreStringWithNullCheck(args[4]);
            xmlHttpRequest->open(method, url, async, user, passwd, ec);
        } else
            xmlHttpRequest->open(method, url, async, user, ec);
    } else
        xmlHttpRequest->open(method, url, async, ec);

    if (ec)
        return throwError(ec);

    return v8::Undefined();
}

static bool IsDocumentType(v8::Handle<v8::Value> value)
{
    // FIXME: add other document types.
    return V8Document::HasInstance(value) || V8HTMLDocument::HasInstance(value);
}

v8::Handle<v8::Value> V8XMLHttpRequest::sendCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.send()");
    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(args.Holder());

    ExceptionCode ec = 0;
    if (args.Length() < 1)
        xmlHttpRequest->send(ec);
    else {
        v8::Handle<v8::Value> arg = args[0];
        if (IsDocumentType(arg)) {
            v8::Handle<v8::Object> object = v8::Handle<v8::Object>::Cast(arg);
            Document* document = V8Document::toNative(object);
            ASSERT(document);
            xmlHttpRequest->send(document, ec);
        } else if (V8Blob::HasInstance(arg)) {
            v8::Handle<v8::Object> object = v8::Handle<v8::Object>::Cast(arg);
            Blob* blob = V8Blob::toNative(object);
            ASSERT(blob);
            xmlHttpRequest->send(blob, ec);
        } else
            xmlHttpRequest->send(toWebCoreStringWithNullCheck(arg), ec);
    }

    if (ec)
        return throwError(ec);

    return v8::Undefined();
}

v8::Handle<v8::Value> V8XMLHttpRequest::setRequestHeaderCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.setRequestHeader()");
    if (args.Length() < 2)
        return throwError("Not enough arguments", V8Proxy::SyntaxError);

    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(args.Holder());
    ExceptionCode ec = 0;
    String header = toWebCoreString(args[0]);
    String value = toWebCoreString(args[1]);
    xmlHttpRequest->setRequestHeader(header, value, ec);
    if (ec)
        return throwError(ec);
    return v8::Undefined();
}

v8::Handle<v8::Value> V8XMLHttpRequest::getResponseHeaderCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.getResponseHeader()");
    if (args.Length() < 1)
        return throwError("Not enough arguments", V8Proxy::SyntaxError);

    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(args.Holder());
    ExceptionCode ec = 0;
    String header = toWebCoreString(args[0]);
    String result = xmlHttpRequest->getResponseHeader(header, ec);
    if (ec)
        return throwError(ec);
    return v8StringOrNull(result);
}

v8::Handle<v8::Value> V8XMLHttpRequest::overrideMimeTypeCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.overrideMimeType()");
    if (args.Length() < 1)
        return throwError("Not enough arguments", V8Proxy::SyntaxError);

    XMLHttpRequest* xmlHttpRequest = V8XMLHttpRequest::toNative(args.Holder());
    String value = toWebCoreString(args[0]);
    xmlHttpRequest->overrideMimeType(value);
    return v8::Undefined();
}

v8::Handle<v8::Value> V8XMLHttpRequest::dispatchEventCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.dispatchEvent()");
    return v8::Undefined();
}

} // namespace WebCore
