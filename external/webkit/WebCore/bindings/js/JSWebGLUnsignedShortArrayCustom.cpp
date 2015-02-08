

#include "config.h"

#if ENABLE(3D_CANVAS)

#include "JSWebGLArrayHelper.h"
#include "JSWebGLUnsignedShortArray.h"

#include "WebGLUnsignedShortArray.h"

using namespace JSC;

namespace WebCore {

void JSWebGLUnsignedShortArray::indexSetter(JSC::ExecState* exec, unsigned index, JSC::JSValue value)
{
    impl()->set(index, static_cast<unsigned short>(value.toInt32(exec)));
}

JSC::JSValue toJS(JSC::ExecState* exec, JSDOMGlobalObject* globalObject, WebGLUnsignedShortArray* object)
{
    return getDOMObjectWrapper<JSWebGLUnsignedShortArray>(exec, globalObject, object);
}

JSC::JSValue JSWebGLUnsignedShortArray::set(JSC::ExecState* exec, JSC::ArgList const& args)
{
    if (args.size() > 2)
        return throwError(exec, SyntaxError);

    if (args.size() == 2 && args.at(0).isInt32()) {
        // void set(in unsigned long index, in long value);
        unsigned index = args.at(0).toUInt32(exec);
        impl()->set(index, static_cast<unsigned short>(args.at(1).toInt32(exec)));
        return jsUndefined();
    }

    WebGLUnsignedShortArray* array = toWebGLUnsignedShortArray(args.at(0));
    if (array) {
        // void set(in WebGLUnsignedShortArray array, [Optional] in unsigned long offset);
        unsigned offset = 0;
        if (args.size() == 2)
            offset = args.at(1).toInt32(exec);
        ExceptionCode ec = 0;
        impl()->set(array, offset, ec);
        setDOMException(exec, ec);
        return jsUndefined();
    }

    return setWebGLArrayFromArray(exec, impl(), args);
}

} // namespace WebCore

#endif // ENABLE(3D_CANVAS)
