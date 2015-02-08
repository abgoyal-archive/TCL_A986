

#include "config.h"

#if ENABLE(EVENTSOURCE)
#include "V8EventSource.h"

#include "EventSource.h"

#include "V8Binding.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8EventSource::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.EventSource.addEventListener()");
    EventSource* eventSource = V8EventSource::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(eventSource, args[1], false, ListenerFindOrCreate);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        eventSource->addEventListener(type, listener, useCapture);

        createHiddenDependency(args.Holder(), args[1], cacheIndex);
    }
    return v8::Undefined();
}

v8::Handle<v8::Value> V8EventSource::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.EventSource.removeEventListener()");
    EventSource* eventSource = V8EventSource::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(eventSource, args[1], false, ListenerFindOnly);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        eventSource->removeEventListener(type, listener.get(), useCapture);

        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
    }

    return v8::Undefined();
}

} // namespace WebCore

#endif // ENABLE(EVENTSOURCE)
