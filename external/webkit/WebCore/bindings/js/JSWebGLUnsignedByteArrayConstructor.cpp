

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLUnsignedByteArrayConstructor.h"

#include "Document.h"
#include "WebGLUnsignedByteArray.h"
#include "JSWebGLArrayBuffer.h"
#include "JSWebGLArrayBufferConstructor.h"
#include "JSWebGLUnsignedByteArray.h"
#include <runtime/Error.h>

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLUnsignedByteArrayConstructor::s_info = { "WebGLUnsignedByteArrayConstructor", &JSWebGLArray::s_info, 0, 0 };

JSWebGLUnsignedByteArrayConstructor::JSWebGLUnsignedByteArrayConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLUnsignedByteArrayConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLUnsignedByteArrayPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasUnsignedByteArray(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLUnsignedByteArrayConstructor* jsConstructor = static_cast<JSWebGLUnsignedByteArrayConstructor*>(constructor);
    RefPtr<WebGLUnsignedByteArray> array = static_cast<WebGLUnsignedByteArray*>(construct<WebGLUnsignedByteArray, unsigned char>(exec, args).get());
    return asObject(toJS(exec, jsConstructor->globalObject(), array.get()));
}

JSC::ConstructType JSWebGLUnsignedByteArrayConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasUnsignedByteArray;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
