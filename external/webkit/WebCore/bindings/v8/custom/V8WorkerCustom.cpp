

#include "config.h"

#if ENABLE(WORKERS)
#include "V8Worker.h"

#include "Worker.h"

#include "ExceptionCode.h"
#include "Frame.h"
#include "SerializedScriptValue.h"
#include "V8Binding.h"
#include "V8MessagePortCustom.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8Worker::constructorCallback(const v8::Arguments& args)
{
    INC_STATS(L"DOM.Worker.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    if (!args.Length())
        return throwError("Not enough arguments", V8Proxy::SyntaxError);

    v8::TryCatch tryCatch;
    v8::Handle<v8::String> scriptUrl = args[0]->ToString();
    if (tryCatch.HasCaught())
        return throwError(tryCatch.Exception());

    if (scriptUrl.IsEmpty())
        return v8::Undefined();

    // Get the script execution context.
    ScriptExecutionContext* context = getScriptExecutionContext();
    if (!context)
        return v8::Undefined();

    // Create the worker object.
    // Note: it's OK to let this RefPtr go out of scope because we also call setDOMWrapper(), which effectively holds a reference to obj.
    ExceptionCode ec = 0;
    RefPtr<Worker> obj = Worker::create(toWebCoreString(scriptUrl), context, ec);
    if (ec)
        return throwError(ec);

    // Setup the standard wrapper object internal fields.
    v8::Handle<v8::Object> wrapperObject = args.Holder();
    V8DOMWrapper::setDOMWrapper(wrapperObject, V8ClassIndex::WORKER, obj.get());

    obj->ref();
    V8DOMWrapper::setJSWrapperForActiveDOMObject(obj.get(), v8::Persistent<v8::Object>::New(wrapperObject));

    return wrapperObject;
}

v8::Handle<v8::Value> V8Worker::postMessageCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.Worker.postMessage");
    Worker* worker = V8Worker::toNative(args.Holder());
    RefPtr<SerializedScriptValue> message = SerializedScriptValue::create(args[0]);
    MessagePortArray portArray;
    if (args.Length() > 1) {
        if (!getMessagePortArray(args[1], portArray))
            return v8::Undefined();
    }
    ExceptionCode ec = 0;
    worker->postMessage(message.release(), &portArray, ec);
    return throwError(ec);
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
