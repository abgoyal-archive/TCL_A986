

#include "config.h"
#include "JSCallbackConstructor.h"

#include "APIShims.h"
#include "APICast.h"
#include <runtime/JSGlobalObject.h>
#include <runtime/JSLock.h>
#include <runtime/ObjectPrototype.h>
#include <wtf/Vector.h>

namespace JSC {

const ClassInfo JSCallbackConstructor::info = { "CallbackConstructor", 0, 0, 0 };

JSCallbackConstructor::JSCallbackConstructor(NonNullPassRefPtr<Structure> structure, JSClassRef jsClass, JSObjectCallAsConstructorCallback callback)
    : JSObject(structure)
    , m_class(jsClass)
    , m_callback(callback)
{
    if (m_class)
        JSClassRetain(jsClass);
}

JSCallbackConstructor::~JSCallbackConstructor()
{
    if (m_class)
        JSClassRelease(m_class);
}

static JSObject* constructJSCallback(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSContextRef ctx = toRef(exec);
    JSObjectRef constructorRef = toRef(constructor);

    JSObjectCallAsConstructorCallback callback = static_cast<JSCallbackConstructor*>(constructor)->callback();
    if (callback) {
        int argumentCount = static_cast<int>(args.size());
        Vector<JSValueRef, 16> arguments(argumentCount);
        for (int i = 0; i < argumentCount; i++)
            arguments[i] = toRef(exec, args.at(i));

        JSValueRef exception = 0;
        JSObjectRef result;
        {
            APICallbackShim callbackShim(exec);
            result = callback(ctx, constructorRef, argumentCount, arguments.data(), &exception);
        }
        if (exception)
            exec->setException(toJS(exec, exception));
        return toJS(result);
    }
    
    return toJS(JSObjectMake(ctx, static_cast<JSCallbackConstructor*>(constructor)->classRef(), 0));
}

ConstructType JSCallbackConstructor::getConstructData(ConstructData& constructData)
{
    constructData.native.function = constructJSCallback;
    return ConstructTypeHost;
}

} // namespace JSC
