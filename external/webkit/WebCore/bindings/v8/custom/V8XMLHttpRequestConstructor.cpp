

#include "config.h"
#include "V8XMLHttpRequest.h"

#include "Frame.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8XMLHttpRequest::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.XMLHttpRequest.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.", V8Proxy::TypeError);

    // Expect no parameters.
    // Allocate a XMLHttpRequest object as its internal field.
    ScriptExecutionContext* context = getScriptExecutionContext();
    if (!context)
        return throwError("XMLHttpRequest constructor's associated context is not available", V8Proxy::ReferenceError);
    RefPtr<XMLHttpRequest> xmlHttpRequest = XMLHttpRequest::create(context);
    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::ToInt(V8ClassIndex::XMLHTTPREQUEST), xmlHttpRequest.get());

    // Add object to the wrapper map.
    xmlHttpRequest->ref();
    V8DOMWrapper::setJSWrapperForActiveDOMObject(xmlHttpRequest.get(), v8::Persistent<v8::Object>::New(args.Holder()));
    return args.Holder();
}

} // namespace WebCore
