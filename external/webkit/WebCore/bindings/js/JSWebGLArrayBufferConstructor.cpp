

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLArrayBufferConstructor.h"

#include "Document.h"
#include "WebGLArrayBuffer.h"
#include "JSWebGLArrayBuffer.h"

namespace WebCore {

using namespace JSC;

const ClassInfo JSWebGLArrayBufferConstructor::s_info = { "WebGLArrayBufferConstructor", 0, 0, 0 };

JSWebGLArrayBufferConstructor::JSWebGLArrayBufferConstructor(ExecState* exec, JSDOMGlobalObject* globalObject)
    : DOMConstructorObject(JSWebGLArrayBufferConstructor::createStructure(globalObject->objectPrototype()), globalObject)
{
    putDirect(exec->propertyNames().prototype, JSWebGLArrayBufferPrototype::self(exec, globalObject), None);
    putDirect(exec->propertyNames().length, jsNumber(exec, 2), ReadOnly|DontDelete|DontEnum);
}

static JSObject* constructCanvasArrayBuffer(ExecState* exec, JSObject* constructor, const ArgList& args)
{
    JSWebGLArrayBufferConstructor* jsConstructor = static_cast<JSWebGLArrayBufferConstructor*>(constructor);

    unsigned int size = 0;
    if (args.size() == 1) {
        size = (unsigned int)args.at(0).toInt32(exec);
        if (isnan(size))
            size = 0;
    }
    return asObject(toJS(exec, jsConstructor->globalObject(), WebGLArrayBuffer::create(size)));
}

JSC::ConstructType JSWebGLArrayBufferConstructor::getConstructData(JSC::ConstructData& constructData)
{
    constructData.native.function = constructCanvasArrayBuffer;
    return ConstructTypeHost;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
