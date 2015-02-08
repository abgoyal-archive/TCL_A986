

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLUnsignedByteArray.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8WebGLArrayCustom.h"
#include "V8WebGLUnsignedByteArray.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLUnsignedByteArray::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedByteArray.Contructor");

    return constructWebGLArray<WebGLUnsignedByteArray>(args, V8ClassIndex::ToInt(V8ClassIndex::WEBGLUNSIGNEDBYTEARRAY));
}

v8::Handle<v8::Value> V8WebGLUnsignedByteArray::getCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedByteArray.get()");
    return getWebGLArrayElement<WebGLUnsignedByteArray, unsigned char>(args, V8ClassIndex::WEBGLUNSIGNEDBYTEARRAY);
}

v8::Handle<v8::Value> V8WebGLUnsignedByteArray::setCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedByteArray.set()");
    return setWebGLArray<WebGLUnsignedByteArray, V8WebGLUnsignedByteArray>(args, V8ClassIndex::WEBGLUNSIGNEDBYTEARRAY);
}

v8::Handle<v8::Value> toV8(WebGLUnsignedByteArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8WebGLUnsignedByteArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToExternalArrayData(impl->baseAddress(), v8::kExternalUnsignedByteArray, impl->length());
    return wrapper;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
