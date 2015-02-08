

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLByteArray.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8WebGLArrayCustom.h"
#include "V8WebGLByteArray.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLByteArray::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLByteArray.Contructor");

    return constructWebGLArray<WebGLByteArray>(args, V8ClassIndex::ToInt(V8ClassIndex::WEBGLBYTEARRAY));
}

v8::Handle<v8::Value> V8WebGLByteArray::getCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLByteArray.get()");
    return getWebGLArrayElement<WebGLByteArray, signed char>(args, V8ClassIndex::WEBGLBYTEARRAY);
}

v8::Handle<v8::Value> V8WebGLByteArray::setCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLByteArray.set()");
    return setWebGLArray<WebGLByteArray, V8WebGLByteArray>(args, V8ClassIndex::WEBGLBYTEARRAY);
}

v8::Handle<v8::Value> toV8(WebGLByteArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8WebGLByteArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToExternalArrayData(impl->baseAddress(), v8::kExternalByteArray, impl->length());
    return wrapper;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
