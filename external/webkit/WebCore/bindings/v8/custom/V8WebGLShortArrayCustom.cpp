

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLShortArray.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8WebGLArrayCustom.h"
#include "V8WebGLShortArray.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLShortArray::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLShortArray.Contructor");

    return constructWebGLArray<WebGLShortArray>(args, V8ClassIndex::ToInt(V8ClassIndex::WEBGLSHORTARRAY));
}

v8::Handle<v8::Value> V8WebGLShortArray::getCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLShortArray.get()");
    return getWebGLArrayElement<WebGLShortArray, short>(args, V8ClassIndex::WEBGLSHORTARRAY);
}

v8::Handle<v8::Value> V8WebGLShortArray::setCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLShortArray.set()");
    return setWebGLArray<WebGLShortArray, V8WebGLShortArray>(args, V8ClassIndex::WEBGLSHORTARRAY);
}

v8::Handle<v8::Value> toV8(WebGLShortArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8WebGLShortArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToExternalArrayData(impl->baseAddress(), v8::kExternalShortArray, impl->length());
    return wrapper;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
