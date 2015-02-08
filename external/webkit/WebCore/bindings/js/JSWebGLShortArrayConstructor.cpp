

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLShortArrayConstructor.h"

#include "Document.h"
#include "WebGLShortArray.h"
#include "JSWebGLArray.h"
#include "JSWebGLArrayBuffer.h"
#include "JSWebGLArrayBufferConstructor.h"
#include "JSWebGLShortArray.h"
#include <runtime/Error.h>

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLShortArrayConstructor::s_info = { "WebGLShortArrayConstructor", &JSWebGLArray::s_info, 0, 0 };

JSWebGLShortArrayConstructor::JSWebGLShortArrayConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLShortArrayConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLShortArrayPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasShortArray(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLShortArrayConstructor* jsConstructor = static_cast<JSWebGLShortArrayConstructor*>(constructor);
    RefPtr<WebGLShortArray> array = static_cast<WebGLShortArray*>(construct<WebGLShortArray, short>(exec, args).get());
    return asObject(toJS(exec, jsConstructor->globalObject(), array.get()));
}

JSC::ConstructType JSWebGLShortArrayConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasShortArray;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
