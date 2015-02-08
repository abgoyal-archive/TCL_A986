

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "config.h"
#include "JSWebGLArray.h"
#include "JSWebGLByteArray.h"
#include "JSWebGLUnsignedByteArray.h"
#include "JSWebGLShortArray.h"
#include "JSWebGLUnsignedShortArray.h"
#include "JSWebGLIntArray.h"
#include "JSWebGLUnsignedIntArray.h"
#include "JSWebGLFloatArray.h"

#include "WebGLArray.h"

using namespace JSC;

namespace WebCore {

JSC::JSValue toJS(JSC::ExecState* exec, JSDOMGlobalObject* globalObject, WebGLArray* object)
{
    if (!object)
        return jsUndefined();
        
    if (object) {
        if (object->isFloatArray())
            return getDOMObjectWrapper<JSWebGLFloatArray>(exec, globalObject, static_cast<WebGLFloatArray*>(object));
        if (object->isUnsignedByteArray())
            return getDOMObjectWrapper<JSWebGLUnsignedByteArray>(exec, globalObject, static_cast<WebGLUnsignedByteArray*>(object));
        if (object->isByteArray())
            return getDOMObjectWrapper<JSWebGLByteArray>(exec, globalObject, static_cast<WebGLByteArray*>(object));
        if (object->isIntArray())
            return getDOMObjectWrapper<JSWebGLIntArray>(exec, globalObject, static_cast<WebGLIntArray*>(object));
        if (object->isUnsignedIntArray())
            return getDOMObjectWrapper<JSWebGLUnsignedIntArray>(exec, globalObject, static_cast<WebGLUnsignedIntArray*>(object));
        if (object->isShortArray())
            return getDOMObjectWrapper<JSWebGLShortArray>(exec, globalObject, static_cast<WebGLShortArray*>(object));
        if (object->isUnsignedShortArray())
            return getDOMObjectWrapper<JSWebGLUnsignedShortArray>(exec, globalObject, static_cast<WebGLUnsignedShortArray*>(object));
    }
    return jsUndefined();
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
