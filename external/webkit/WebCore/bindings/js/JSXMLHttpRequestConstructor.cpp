

#include "config.h"
#include "JSXMLHttpRequestConstructor.h"

#include "JSXMLHttpRequest.h"
#include "ScriptExecutionContext.h"
#include "XMLHttpRequest.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

ASSERT_CLASS_FITS_IN_CELL(JSXMLHttpRequestConstructor);

const ClassInfo JSXMLHttpRequestConstructor::s_info = { "XMLHttpRequestConstructor", 0, 0, 0 };

JSXMLHttpRequestConstructor::JSXMLHttpRequestConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSXMLHttpRequestConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSXMLHttpRequestPrototype::self(exec, globalObject), None);
}

static JSObject* constructXMLHttpRequest(ExecState* exec, JSObject* constructor, const ArgList&)
{
    JSXMLHttpRequestConstructor* jsConstructor = static_cast<JSXMLHttpRequestConstructor*>(constructor);
    ScriptExecutionContext* context = jsConstructor->scriptExecutionContext();
    if (!context)
        return throwError(exec, ReferenceError, "XMLHttpRequest constructor associated document is unavailable");

    RefPtr<XMLHttpRequest> xmlHttpRequest = XMLHttpRequest::create(context);
    return CREATE_DOM_OBJECT_WRAPPER(exec, jsConstructor->globalObject(), XMLHttpRequest, xmlHttpRequest.get());
}

ConstructType JSXMLHttpRequestConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructXMLHttpRequest;
    return ConstructTypeHost;
}

} // namespace WebCore
