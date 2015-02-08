

#include "config.h"
#include "JSWebKitCSSMatrixConstructor.h"

#include "WebKitCSSMatrix.h"
#include "JSWebKitCSSMatrix.h"

using namespace JSC;

namespace WebCore {

const ClassInfo JSWebKitCSSMatrixConstructor::s_info = { "WebKitCSSMatrixConstructor", 0, 0, 0 };

JSWebKitCSSMatrixConstructor::JSWebKitCSSMatrixConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebKitCSSMatrixConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebKitCSSMatrixPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 1), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructWebKitCSSMatrix(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebKitCSSMatrixConstructor* jsConstructor = static_cast<JSWebKitCSSMatrixConstructor*>(constructor);
    String s;
    if (args.size() >= 1)
        s = args.at(0).toString(exec);
    
    ExceptionCode ec = 0;
    RefPtr<WebKitCSSMatrix> matrix = WebKitCSSMatrix::create(s, ec);
    setDOMException(exec, ec);
    return CREATE_DOM_OBJECT_WRAPPER(exec, jsConstructor->globalObject(), WebKitCSSMatrix, matrix.get());
}

ConstructType JSWebKitCSSMatrixConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructWebKitCSSMatrix;
    return ConstructTypeHost;
}

} // namespace WebCore
