

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLIntArray.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8WebGLArrayCustom.h"
#include "V8WebGLIntArray.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLIntArray::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLIntArray.Contructor");

    return constructWebGLArray<WebGLIntArray>(args, V8ClassIndex::ToInt(V8ClassIndex::WEBGLINTARRAY));
}

v8::Handle<v8::Value> V8WebGLIntArray::getCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLIntArray.get()");
    return getWebGLArrayElement<WebGLIntArray, int>(args, V8ClassIndex::WEBGLINTARRAY);
}

v8::Handle<v8::Value> V8WebGLIntArray::setCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLIntArray.set()");
    return setWebGLArray<WebGLIntArray, V8WebGLIntArray>(args, V8ClassIndex::WEBGLINTARRAY);
}

v8::Handle<v8::Value> toV8(WebGLIntArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8WebGLIntArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToExternalArrayData(impl->baseAddress(), v8::kExternalIntArray, impl->length());
    return wrapper;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
