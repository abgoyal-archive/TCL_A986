

#include "config.h"

#if ENABLE(SHARED_WORKERS)

#include "JSSharedWorkerConstructor.h"

#include "JSDOMWindowCustom.h"
#include "JSSharedWorker.h"
#include "SharedWorker.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

const ClassInfo JSSharedWorkerConstructor::s_info = { "SharedWorkerConstructor", 0, 0, 0 };

JSSharedWorkerConstructor::JSSharedWorkerConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSSharedWorkerConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSSharedWorkerPrototype::self(exec, globalObject), None);
    // Host functions have a length property describing the number of expected arguments.
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructSharedWorker(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSSharedWorkerConstructor* jsConstructor = static_cast<JSSharedWorkerConstructor*>(constructor);

    if (args.size() < 1)
        return throwError(exec, SyntaxError, "Not enough arguments");

    UString scriptURL = args.at(0).toString(exec);
    UString name;
    if (args.size() > 1)
        name = args.at(1).toString(exec);

    if (exec->hadException())
        return 0;

    // FIXME: We need to use both the dynamic scope and the lexical scope (dynamic scope for resolving the worker URL)
    DOMWindow* window = asJSDOMWindow(exec->lexicalGlobalObject())->impl();
    ExceptionCode ec = 0;
    RefPtr<SharedWorker> worker = SharedWorker::create(scriptURL, name, window->document(), ec);
    setDOMException(exec, ec);

    return asObject(toJS(exec, jsConstructor->globalObject(), worker.release()));
}

ConstructType JSSharedWorkerConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructSharedWorker;
    return ConstructTypeHost;
}


} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)
