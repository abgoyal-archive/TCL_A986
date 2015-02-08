

#include "config.h"

#if ENABLE(WORKERS)
#include "V8DedicatedWorkerContext.h"

#include "DedicatedWorkerContext.h"
#include "WorkerContextExecutionProxy.h"
#include "V8Binding.h"
#include "V8MessagePortCustom.h"
#include "V8Proxy.h"
#include "V8WorkerContextEventListener.h"

namespace WebCore {

v8::Handle<v8::Value> V8DedicatedWorkerContext::postMessageCallback(const v8::Arguments& args)
{
    INC_STATS(L"DOM.DedicatedWorkerContext.postMessage");
    DedicatedWorkerContext* workerContext = V8DedicatedWorkerContext::toNative(args.Holder());
    RefPtr<SerializedScriptValue> message = SerializedScriptValue::create(args[0]);
    MessagePortArray portArray;
    if (args.Length() > 1) {
        if (!getMessagePortArray(args[1], portArray))
            return v8::Undefined();
    }
    ExceptionCode ec = 0;
    workerContext->postMessage(message.release(), &portArray, ec);
    return throwError(ec);
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
