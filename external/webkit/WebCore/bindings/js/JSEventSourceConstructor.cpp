

#include "config.h"

#if ENABLE(EVENTSOURCE)

#include "JSEventSourceConstructor.h"

#include "EventSource.h"
#include "ExceptionCode.h"
#include "JSEventSource.h"
#include "ScriptExecutionContext.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

ASSERT_CLASS_FITS_IN_CELL(JSEventSourceConstructor);

const ClassInfo JSEventSourceConstructor::s_info = { "EventSourceContructor", 0, 0, 0 };

JSEventSourceConstructor::JSEventSourceConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSEventSourceConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSEventSourcePrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 1), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructEventSource(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    if (args.size() < 1)
        return throwError(exec, SyntaxError, "Not enough arguments");

    UString url = args.at(0).toString(exec);
    if (exec->hadException())
        return 0;

    JSEventSourceConstructor* jsConstructor =  static_cast<JSEventSourceConstructor*>(constructor);
    ScriptExecutionContext* context = jsConstructor->scriptExecutionContext();
    if (!context)
        return throwError(exec, ReferenceError, "EventSource constructor associated document is unavailable");

    ExceptionCode ec = 0;
    RefPtr<EventSource> eventSource = EventSource::create(url, context, ec);
    if (ec) {
        setDOMException(exec, ec);
        return 0;
    }

    return asObject(toJS(exec, jsConstructor->globalObject(), eventSource.release()));
}

ConstructType JSEventSourceConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructEventSource;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(EVENTSOURCE)
