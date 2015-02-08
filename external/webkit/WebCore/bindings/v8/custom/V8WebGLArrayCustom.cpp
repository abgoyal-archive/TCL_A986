

#include "config.h"

#if ENABLE(3D_CANVAS)
#include "V8WebGLArray.h"

#include "V8WebGLByteArray.h"
#include "V8WebGLFloatArray.h"
#include "V8WebGLIntArray.h"
#include "V8WebGLShortArray.h"
#include "V8WebGLUnsignedByteArray.h"
#include "V8WebGLUnsignedIntArray.h"
#include "V8WebGLUnsignedShortArray.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(WebGLArray* impl)
{
    if (!impl)
        return v8::Null();
    if (impl->isByteArray())
        return toV8(static_cast<WebGLByteArray*>(impl));
    if (impl->isFloatArray())
        return toV8(static_cast<WebGLFloatArray*>(impl));
    if (impl->isIntArray())
        return toV8(static_cast<WebGLIntArray*>(impl));
    if (impl->isShortArray())
        return toV8(static_cast<WebGLShortArray*>(impl));
    if (impl->isUnsignedByteArray())
        return toV8(static_cast<WebGLUnsignedByteArray*>(impl));
    if (impl->isUnsignedIntArray())
        return toV8(static_cast<WebGLUnsignedIntArray*>(impl));
    if (impl->isUnsignedShortArray())
        return toV8(static_cast<WebGLUnsignedShortArray*>(impl));
    return v8::Handle<v8::Value>();
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
