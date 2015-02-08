

#include "config.h"

#if ENABLE(SHARED_WORKERS)

#include "V8SharedWorker.h"

#include "ExceptionCode.h"
#include "Frame.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8Utilities.h"
#include "WorkerContext.h"
#include "WorkerContextExecutionProxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8SharedWorker::constructorCallback(const v8::Arguments& args)
{
    INC_STATS(L"DOM.SharedWorker.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    if (!args.Length())
        return throwError("Not enough arguments", V8Proxy::SyntaxError);

    v8::TryCatch tryCatch;
    v8::Handle<v8::String> scriptUrl = args[0]->ToString();
    String name;
    if (args.Length() > 1)
        name = toWebCoreString(args[1]->ToString());

    if (tryCatch.HasCaught())
        return throwError(tryCatch.Exception());

    if (scriptUrl.IsEmpty())
        return v8::Undefined();

    // Get the script execution context.
    ScriptExecutionContext* context = getScriptExecutionContext();
    if (!context)
        return v8::Undefined();

    // Create the SharedWorker object.
    // Note: it's OK to let this RefPtr go out of scope because we also call SetDOMWrapper(), which effectively holds a reference to obj.
    ExceptionCode ec = 0;
    RefPtr<SharedWorker> obj = SharedWorker::create(toWebCoreString(scriptUrl), name, context, ec);
    if (ec)
        return throwError(ec);

    // Setup the standard wrapper object internal fields.
    v8::Handle<v8::Object> wrapperObject = args.Holder();
    V8DOMWrapper::setDOMWrapper(wrapperObject, V8ClassIndex::SHAREDWORKER, obj.get());

    obj->ref();
    V8DOMWrapper::setJSWrapperForActiveDOMObject(obj.get(), v8::Persistent<v8::Object>::New(wrapperObject));

    return wrapperObject;
}

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)
