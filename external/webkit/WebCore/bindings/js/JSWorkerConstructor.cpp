

#include "config.h"

#if ENABLE(WORKERS)

#include "JSWorkerConstructor.h"

#include "Document.h"
#include "ExceptionCode.h"
#include "JSDOMWindowCustom.h"
#include "JSWorker.h"
#include "Worker.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

const ClassInfo JSWorkerConstructor::s_info = { "WorkerConstructor", 0, 0, 0 };

JSWorkerConstructor::JSWorkerConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWorkerConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWorkerPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 1), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructWorker(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWorkerConstructor* jsConstructor = static_cast<JSWorkerConstructor*>(constructor);

    if (args.size() == 0)
        return throwError(exec, SyntaxError, "Not enough arguments");

    UString scriptURL = args.at(0).toString(exec);
    if (exec->hadException())
        return 0;

    // See section 4.8.2 step 14 of WebWorkers for why this is the lexicalGlobalObject. 
    DOMWindow* window = asJSDOMWindow(exec->lexicalGlobalObject())->impl();

    ExceptionCode ec = 0;
    RefPtr<Worker> worker = Worker::create(scriptURL, window->document(), ec);
    if (ec) {
        setDOMException(exec, ec);
        return 0;
    }

    return asObject(toJS(exec, jsConstructor->globalObject(), worker.release()));
}

ConstructType JSWorkerConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructWorker;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(WORKERS)
