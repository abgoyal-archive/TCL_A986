

#include "config.h"

#if ENABLE(WORKERS)
#include "V8AbstractWorker.h"

#include "AbstractWorker.h"
#include "ExceptionCode.h"
#include "ScriptExecutionContext.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8AbstractWorker::addEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS(L"DOM.AbstractWorker.addEventListener()");
    AbstractWorker* worker = V8AbstractWorker::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(worker, args[1], false, ListenerFindOrCreate);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        worker->addEventListener(type, listener, useCapture);

        createHiddenDependency(args.Holder(), args[1], cacheIndex);
    }
    return v8::Undefined();
}

v8::Handle<v8::Value> V8AbstractWorker::removeEventListenerCallback(const v8::Arguments& args)
{
    INC_STATS(L"DOM.AbstractWorker.removeEventListener()");
    AbstractWorker* worker = V8AbstractWorker::toNative(args.Holder());

    RefPtr<EventListener> listener = V8DOMWrapper::getEventListener(worker, args[1], false, ListenerFindOnly);
    if (listener) {
        String type = toWebCoreString(args[0]);
        bool useCapture = args[2]->BooleanValue();
        worker->removeEventListener(type, listener.get(), useCapture);

        removeHiddenDependency(args.Holder(), args[1], cacheIndex);
    }

    return v8::Undefined();
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
