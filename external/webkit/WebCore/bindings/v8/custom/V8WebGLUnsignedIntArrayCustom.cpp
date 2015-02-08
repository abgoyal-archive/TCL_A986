

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLUnsignedIntArray.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8WebGLArrayCustom.h"
#include "V8WebGLUnsignedIntArray.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLUnsignedIntArray::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedIntArray.Contructor");

    return constructWebGLArray<WebGLUnsignedIntArray>(args, V8ClassIndex::ToInt(V8ClassIndex::WEBGLUNSIGNEDINTARRAY));
}

v8::Handle<v8::Value> V8WebGLUnsignedIntArray::getCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedIntArray.get()");
    return getWebGLArrayElement<WebGLUnsignedIntArray, unsigned int>(args, V8ClassIndex::WEBGLUNSIGNEDINTARRAY);
}

v8::Handle<v8::Value> V8WebGLUnsignedIntArray::setCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedIntArray.set()");
    return setWebGLArray<WebGLUnsignedIntArray, V8WebGLUnsignedIntArray>(args, V8ClassIndex::WEBGLUNSIGNEDINTARRAY);
}

v8::Handle<v8::Value> toV8(WebGLUnsignedIntArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8WebGLUnsignedIntArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToExternalArrayData(impl->baseAddress(), v8::kExternalUnsignedIntArray, impl->length());
    return wrapper;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
