

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "WebGLArrayBuffer.h"

#include "V8Binding.h"
#include "V8WebGLArrayBuffer.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8WebGLArrayBuffer::constructorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.WebGLArrayBuffer.Constructor");

    if (!args.IsConstructCall())
        return throwError("DOM object constructor cannot be called as a function.");

    // If we return a previously constructed WebGLArrayBuffer,
    // e.g. from the call to WebGLArray.buffer, this code is called
    // with a zero-length argument list. The V8DOMWrapper will then
    // set the internal pointer in the newly-created object.
    // Unfortunately it doesn't look like it's possible to distinguish
    // between this case and that where the user calls "new
    // WebGLArrayBuffer()" from JavaScript. To guard against problems,
    // we always create at least a zero-length WebGLArrayBuffer, even
    // if it is immediately overwritten by the V8DOMWrapper.

    // Supported constructors:
    // WebGLArrayBuffer(n) where n is an integer:
    //   -- create an empty buffer of n bytes

    int argLen = args.Length();
    if (argLen > 1)
        return throwError("Wrong number of arguments specified to constructor (requires 1)");

    int len = 0;
    if (argLen > 0) {
        if (!args[0]->IsInt32())
            return throwError("Argument to WebGLArrayBuffer constructor was not an integer");
        len = toInt32(args[0]);
    }

    RefPtr<WebGLArrayBuffer> buffer = WebGLArrayBuffer::create(len);
    // Transform the holder into a wrapper object for the array.
    V8DOMWrapper::setDOMWrapper(args.Holder(), V8ClassIndex::ToInt(V8ClassIndex::WEBGLARRAYBUFFER), buffer.get());
    return toV8(buffer.release(), args.Holder());
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
