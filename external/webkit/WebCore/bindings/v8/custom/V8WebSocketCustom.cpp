

#include "config.h"

#if ENABLE(WEB_SOCKETS)

#include "V8WebSocket.h"

#include "Frame.h"
#include "Settings.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WebSocket.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebSocket::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebSocket.addEventListener()");
    WebSocket* webSocket = V8WebSocket::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(webSocket, args[1], false, ListenerFindOrCreate);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        webSocket->addEventListener(type, listener, useCapture);

        createHiddenDependency(args.Holder(), args[1], cacheIndex);
    }
    return v8::Undefined();
}

v8::Handle<v8::Value> V8WebSocket::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebSocket.removeEventListener()");
    WebSocket* webSocket = V8WebSocket::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(webSocket, args[1], false, ListenerFindOnly);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        webSocket->removeEventListener(type, listener.get(), useCapture);
        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
    }
    return v8::Undefined();
}

v8::Handle<v8::Value> V8WebSocket::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebSocket.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object custructor cannot be called as a function.");
    if (args.Length() == 0)
        return throwError("Not enough arguments", V8Proxy::SyntaxError);

    v8::TryCatch tryCatch;
    v8::Handle<v8::String> urlstring = args[0]->ToString();
    if (tryCatch.HasCaught())
        return throwError(tryCatch.Exception());
    if (urlstring.IsEmpty())
        return throwError("Empty URL", V8Proxy::SyntaxError);

    // Get the script execution context.
    ScriptExecutionContext* context = getScriptExecutionContext();
    if (!context)
        return throwError("WebSocket constructor's associated frame is not available", V8Proxy::ReferenceError);

    const KURL& url = context->completeURL(toWebCoreString(urlstring));

    RefPtr<WebSocket> webSocket = WebSocket::create(context);
    ExceptionCode ec = 0;

    if (args.Length() < 2)
        webSocket->connect(url, ec);
    else {
        v8::TryCatch tryCatchProtocol;
        v8::Handle<v8::String> protocol = args[1]->ToString();
        if (tryCatchProtocol.HasCaught())
            return throwError(tryCatchProtocol.Exception());
        webSocket->connect(url, toWebCoreString(protocol), ec);
    }
    if (ec)
        return throwError(ec);

    // Setup the standard wrapper object internal fields.
    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::ToInt(V8ClassIndex::WEBSOCKET), webSocket.get());

    // Add object to the wrapper map.
    webSocket->ref();
    V8DOMWrapper::setJSWrapperForActiveDOMObject(webSocket.get(), v8::Persistent<v8::Object>::New(args.Holder()));

    return args.Holder();
}

v8::Handle<v8::Value> V8WebSocket::sendCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebSocket.send()");
    WebSocket* webSocket = V8WebSocket::toNative(args.Holder());

    ExceptionCode ec = 0;
    bool ret = false;
    if (args.Length() < 1)
        return throwError("Not enough arguments", V8Proxy::SyntaxError);
    else {
        String msg = toWebCoreString(args[0]);
        ret = webSocket->send(msg, ec);
    }
    if (ec)
        return throwError(ec);
    return v8Boolean(ret);
}

}  // namespace WebCore

#endif  // ENABLE(WEB_SOCKETS)
