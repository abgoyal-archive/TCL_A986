

#include "config.h"
#include "V8DOMApplicationCache.h"

#if ENABLE(OFFLINE_WEB_APPLICATIONS)

#include "ApplicationCacheHost.h"
#include "DOMApplicationCache.h"
#include "V8Binding.h"
#include "V8Document.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

// Handles appcache.addEventListner(name, func, capture) method calls
v8::Handle<v8::Value> V8DOMApplicationCache::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOMApplicationCache.addEventListener()");
    DOMApplicationCache* appcache = V8DOMApplicationCache::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(appcache, args[1], false, ListenerFindOrCreate);
    if (listener) {
        createHiddenDependency(args.Holder(), args[1], cacheIndex);
        String eventType = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        appcache->addEventListener(eventType, listener, useCapture);
    }
    return v8::Undefined();
}

// Handles appcache.removeEventListner(name, func, capture) method calls
v8::Handle<v8::Value> V8DOMApplicationCache::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOMApplicationCache.removeEventListener()");
    DOMApplicationCache* appcache = V8DOMApplicationCache::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(appcache, args[1], false, ListenerFindOnly);
    if (listener) {
        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
        String eventType = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        appcache->removeEventListener(eventType, listener.get(), useCapture);
    }
    return v8::Undefined();
}

} // namespace WebCore

#endif  // ENABLE(OFFLINE_WEB_APPLICATIONS)
