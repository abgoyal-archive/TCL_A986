

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLFloatArray.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8WebGLArrayCustom.h"
#include "V8WebGLFloatArray.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLFloatArray::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLFloatArray.Contructor");

    return constructWebGLArray<WebGLFloatArray>(args, V8ClassIndex::ToInt(V8ClassIndex::WEBGLFLOATARRAY));
}

v8::Handle<v8::Value> V8WebGLFloatArray::getCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLFloatArray.get()");
    return getWebGLArrayElement<WebGLFloatArray, float>(args, V8ClassIndex::WEBGLFLOATARRAY);
}

v8::Handle<v8::Value> V8WebGLFloatArray::setCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLFloatArray.set()");
    return setWebGLArray<WebGLFloatArray, V8WebGLFloatArray>(args, V8ClassIndex::WEBGLFLOATARRAY);
}

v8::Handle<v8::Value> toV8(WebGLFloatArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8WebGLFloatArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToExternalArrayData(impl->baseAddress(), v8::kExternalFloatArray, impl->length());
    return wrapper;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
