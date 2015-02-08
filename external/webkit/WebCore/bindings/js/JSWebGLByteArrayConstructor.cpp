

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLByteArrayConstructor.h"

#include "Document.h"
#include "WebGLByteArray.h"
#include "JSWebGLArrayBuffer.h"
#include "JSWebGLArrayBufferConstructor.h"
#include "JSWebGLByteArray.h"
#include <runtime/Error.h>

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLByteArrayConstructor::s_info = { "WebGLByteArrayConstructor", &JSWebGLArray::s_info, 0, 0 };

JSWebGLByteArrayConstructor::JSWebGLByteArrayConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLByteArrayConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLByteArrayPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasByteArray(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLByteArrayConstructor* jsConstructor = static_cast<JSWebGLByteArrayConstructor*>(constructor);
    RefPtr<WebGLByteArray> array = static_cast<WebGLByteArray*>(construct<WebGLByteArray, signed char>(exec, args).get());
    return asObject(toJS(exec, jsConstructor->globalObject(), array.get()));
}

JSC::ConstructType JSWebGLByteArrayConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasByteArray;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
