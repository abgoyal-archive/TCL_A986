

#include "config.h"
#include "JSCanvasRenderingContext.h"

#include "CanvasRenderingContext2D.h"
#include "JSCanvasRenderingContext2D.h"
#if ENABLE(3D_CANVAS)
#include "WebGLRenderingContext.h"
#include "JSWebGLRenderingContext.h"
#endif

using namespace JSC;

namespace WebCore {

JSC::JSValue toJS(JSC::ExecState* exec, JSDOMGlobalObject* globalObject, CanvasRenderingContext* object)
{
    if (!object)
        return jsUndefined();

#if ENABLE(3D_CANVAS)
    if (object->is3d())
        return getDOMObjectWrapper<JSWebGLRenderingContext>(exec, globalObject, static_cast<WebGLRenderingContext*>(object));
#endif
    ASSERT(object->is2d());
    return getDOMObjectWrapper<JSCanvasRenderingContext2D>(exec, globalObject, static_cast<CanvasRenderingContext2D*>(object));
}

} // namespace WebCore
