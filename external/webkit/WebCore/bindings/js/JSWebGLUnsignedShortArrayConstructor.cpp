

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLUnsignedShortArrayConstructor.h"

#include "Document.h"
#include "WebGLUnsignedShortArray.h"
#include "JSWebGLArrayBuffer.h"
#include "JSWebGLArrayBufferConstructor.h"
#include "JSWebGLUnsignedShortArray.h"
#include <runtime/Error.h>

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLUnsignedShortArrayConstructor::s_info = { "WebGLUnsignedShortArrayConstructor", &JSWebGLArray::s_info, 0, 0 };

JSWebGLUnsignedShortArrayConstructor::JSWebGLUnsignedShortArrayConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLUnsignedShortArrayConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLUnsignedShortArrayPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasUnsignedShortArray(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLUnsignedShortArrayConstructor* jsConstructor = static_cast<JSWebGLUnsignedShortArrayConstructor*>(constructor);
    RefPtr<WebGLUnsignedShortArray> array = static_cast<WebGLUnsignedShortArray*>(construct<WebGLUnsignedShortArray, unsigned short>(exec, args).get());
    return asObject(toJS(exec, jsConstructor->globalObject(), array.get()));
}

JSC::ConstructType JSWebGLUnsignedShortArrayConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasUnsignedShortArray;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
