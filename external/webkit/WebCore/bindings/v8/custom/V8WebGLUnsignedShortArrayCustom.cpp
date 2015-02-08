

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"
#include "WebGLUnsignedShortArray.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8WebGLArrayCustom.h"
#include "V8WebGLUnsignedShortArray.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLUnsignedShortArray::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedShortArray.Contructor");

    return constructWebGLArray<WebGLUnsignedShortArray>(args, V8ClassIndex::ToInt(V8ClassIndex::WEBGLUNSIGNEDSHORTARRAY));
}

v8::Handle<v8::Value> V8WebGLUnsignedShortArray::getCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedShortArray.get()");
    return getWebGLArrayElement<WebGLUnsignedShortArray, unsigned short>(args, V8ClassIndex::WEBGLUNSIGNEDSHORTARRAY);
}

v8::Handle<v8::Value> V8WebGLUnsignedShortArray::setCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLUnsignedShortArray.set()");
    return setWebGLArray<WebGLUnsignedShortArray, V8WebGLUnsignedShortArray>(args, V8ClassIndex::WEBGLUNSIGNEDSHORTARRAY);
}

v8::Handle<v8::Value> toV8(WebGLUnsignedShortArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8WebGLUnsignedShortArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToExternalArrayData(impl->baseAddress(), v8::kExternalUnsignedShortArray, impl->length());
    return wrapper;
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
