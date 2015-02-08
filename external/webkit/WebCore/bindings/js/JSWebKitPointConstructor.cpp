

#include "config.h"
#include "JSWebKitPointConstructor.h"

#include "Document.h"
#include "WebKitPoint.h"
#include "JSWebKitPoint.h"

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebKitPointConstructor::s_info = { "WebKitPointConstructor", 0, 0, 0 };

JSWebKitPointConstructor::JSWebKitPointConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebKitPointConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebKitPointPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructWebKitPoint(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebKitPointConstructor* jsConstructor = static_cast<JSWebKitPointConstructor*>(constructor);

    float x = 0;
    float y = 0;
    if (args.size() >= 2) {
        x = (float)args.at(0).toNumber(exec);
        y = (float)args.at(1).toNumber(exec);
        if (isnan(x))
            x = 0;
        if (isnan(y))
            y = 0;
    }
    return asObject(toJS(exec, jsConstructor->globalObject(), WebKitPoint::create(x, y)));
}

JSC::ConstructType JSWebKitPointConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructWebKitPoint;
    return ConstructTypeHost;
}


} // namespace WebCore
