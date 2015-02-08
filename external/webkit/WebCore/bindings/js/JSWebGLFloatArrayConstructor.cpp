

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLFloatArrayConstructor.h"

#include "Document.h"
#include "WebGLFloatArray.h"
#include "JSWebGLArrayBuffer.h"
#include "JSWebGLArrayBufferConstructor.h"
#include "JSWebGLFloatArray.h"
#include <runtime/Error.h>

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLFloatArrayConstructor::s_info = { "WebGLFloatArrayConstructor", &JSWebGLArray::s_info, 0, 0 };

JSWebGLFloatArrayConstructor::JSWebGLFloatArrayConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLFloatArrayConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLFloatArrayPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasFloatArray(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLFloatArrayConstructor* jsConstructor = static_cast<JSWebGLFloatArrayConstructor*>(constructor);
    RefPtr<WebGLFloatArray> array = static_cast<WebGLFloatArray*>(construct<WebGLFloatArray, float>(exec, args).get());
    return asObject(toJS(exec, jsConstructor->globalObject(), array.get()));
}

JSC::ConstructType JSWebGLFloatArrayConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasFloatArray;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
