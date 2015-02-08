

#include "config.h"

#if ENABLE(EVENTSOURCE)
#include "V8EventSource.h"

#include "EventSource.h"
#include "Frame.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8EventSource::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.EventSource.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.", V8Proxy::TypeError);

    // Expect one parameter.
    // Allocate an EventSource object as its internal field.
    ScriptExecutionContext* context = getScriptExecutionContext();
    if (!context)
        return throwError("EventSource constructor's associated context is not available", V8Proxy::ReferenceError);
    if (args.Length() != 1)
        return throwError("EventSource constructor wrong number of parameters", V8Proxy::TypeError);

    ExceptionCode ec = 0;
    String url = toWebCoreString(args[0]);

    RefPtr<EventSource> eventSource = EventSource::create(url, context, ec);
    
    if (ec)
        return throwError(ec);

    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::ToInt(V8ClassIndex::EVENTSOURCE), eventSource.get());

    // Add object to the wrapper map.
    eventSource->ref();
    V8DOMWrapper::setJSWrapperForActiveDOMObject(eventSource.get(), v8::Persistent<v8::Object>::New(args.Holder()));
    return args.Holder();
}

} // namespace WebCore

#endif // ENABLE(EVENTSOURCE)
