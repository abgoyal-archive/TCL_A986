

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLIntArrayConstructor.h"

#include "Document.h"
#include "WebGLIntArray.h"
#include "JSWebGLArrayBuffer.h"
#include "JSWebGLArrayBufferConstructor.h"
#include "JSWebGLIntArray.h"
#include <runtime/Error.h>

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLIntArrayConstructor::s_info = { "WebGLIntArrayConstructor", &JSWebGLArray::s_info, 0, 0 };

JSWebGLIntArrayConstructor::JSWebGLIntArrayConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLIntArrayConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLIntArrayPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasIntArray(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLIntArrayConstructor* jsConstructor = static_cast<JSWebGLIntArrayConstructor*>(constructor);
    RefPtr<WebGLIntArray> array = static_cast<WebGLIntArray*>(construct<WebGLIntArray, int>(exec, args).get());
    return asObject(toJS(exec, jsConstructor->globalObject(), array.get()));
}

JSC::ConstructType JSWebGLIntArrayConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasIntArray;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
