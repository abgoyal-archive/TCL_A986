

#include "config.h"
#include "V8CanvasPixelArray.h"

namespace WebCore {

v8::Handle<v8::Value> toV8(CanvasPixelArray* impl)
{
    if (!impl)
        return v8::Null();
    v8::Handle<v8::Object> wrapper = V8CanvasPixelArray::wrap(impl);
    if (!wrapper.IsEmpty())
        wrapper->SetIndexedPropertiesToPixelData(impl->data()->data(), impl->length());
    return wrapper;
}

} // namespace WebCore
