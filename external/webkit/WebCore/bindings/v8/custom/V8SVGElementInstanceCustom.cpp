

#include <config.h>

#if ENABLE(SVG)
#include "V8SVGElementInstance.h"

#include "EventListener.h"
#include "SVGElementInstance.h"

#include "V8Binding.h"
#include "V8CustomEventListener.h"
#include "V8SVGPODTypeWrapper.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8SVGElementInstance::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SVGElementInstance.AddEventListener()");
    SVGElementInstance* instance = V8SVGElementInstance::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(instance, args[1], false, ListenerFindOrCreate);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        instance->addEventListener(type, listener, useCapture);
        createHiddenDependency(args.Holder(), args[1], cacheIndex);
    }

    return v8::Undefined();
}

v8::Handle<v8::Value> V8SVGElementInstance::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SVGElementInstance.RemoveEventListener()");
    SVGElementInstance* instance = V8SVGElementInstance::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(instance, args[1], false, ListenerFindOnly);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        instance->removeEventListener(type, listener.get(), useCapture);
        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
    }

    return v8::Undefined();
}

} // namespace WebCore

#endif
