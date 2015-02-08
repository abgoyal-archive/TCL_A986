

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLUnsignedIntArrayConstructor.h"

#include "Document.h"
#include "WebGLUnsignedIntArray.h"
#include "JSWebGLArrayBuffer.h"
#include "JSWebGLArrayBufferConstructor.h"
#include "JSWebGLUnsignedIntArray.h"
#include <runtime/Error.h>

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLUnsignedIntArrayConstructor::s_info = { "WebGLUnsignedIntArrayConstructor", &JSWebGLArray::s_info, 0, 0 };

JSWebGLUnsignedIntArrayConstructor::JSWebGLUnsignedIntArrayConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLUnsignedIntArrayConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLUnsignedIntArrayPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasUnsignedIntArray(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLUnsignedIntArrayConstructor* jsConstructor = static_cast<JSWebGLUnsignedIntArrayConstructor*>(constructor);
    RefPtr<WebGLUnsignedIntArray> array = static_cast<WebGLUnsignedIntArray*>(construct<WebGLUnsignedIntArray, unsigned int>(exec, args).get());
    return asObject(toJS(exec, jsConstructor->globalObject(), array.get()));
}

JSC::ConstructType JSWebGLUnsignedIntArrayConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasUnsignedIntArray;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
