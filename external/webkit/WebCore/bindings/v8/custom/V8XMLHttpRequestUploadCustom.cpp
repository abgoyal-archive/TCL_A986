

#include "config.h"
#include "V8XMLHttpRequestUpload.h"

#include "ExceptionCode.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "XMLHttpRequest.h"
#include "XMLHttpRequestUpload.h"

#include <wtf/Assertions.h>

namespace WebCore {

v8::Handle<v8::Value> V8XMLHttpRequestUpload::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequestUpload.addEventListener()");
    XMLHttpRequestUpload* xmlHttpRequestUpload = V8XMLHttpRequestUpload::toNative(args.Holder());

    XMLHttpRequest* xmlHttpRequest = xmlHttpRequestUpload->associatedXMLHttpRequest();

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(xmlHttpRequest, args[1], false, ListenerFindOrCreate);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        xmlHttpRequestUpload->addEventListener(type, listener, useCapture);

        createHiddenDependency(args.Holder(), args[1], cacheIndex);
    }
    return v8::Undefined();
}

v8::Handle<v8::Value> V8XMLHttpRequestUpload::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequestUpload.removeEventListener()");
    XMLHttpRequestUpload* xmlHttpRequestUpload = V8XMLHttpRequestUpload::toNative(args.Holder());

    XMLHttpRequest* xmlHttpRequest = xmlHttpRequestUpload->associatedXMLHttpRequest();

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(xmlHttpRequest, args[1], false, ListenerFindOnly);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        xmlHttpRequestUpload->removeEventListener(type, listener.get(), useCapture);

        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
    }

    return v8::Undefined();
}

v8::Handle<v8::Value> V8XMLHttpRequestUpload::dispatchEventCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequestUpload.dispatchEvent()");
    return throwError(NOT_SUPPORTED_ERR);
}

} // namespace WebCore
