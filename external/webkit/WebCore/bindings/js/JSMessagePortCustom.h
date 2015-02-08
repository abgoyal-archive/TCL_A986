

#ifndef JSMessagePortCustom_h
#define JSMessagePortCustom_h

#include "MessagePort.h"
#include <runtime/JSValue.h>

namespace WebCore {

    typedef int ExceptionCode;

    class String;

    // Helper function which pulls the values out of a JS sequence and into a MessagePortArray.
    // Also validates the elements per sections 4.1.13 and 4.1.15 of the WebIDL spec and section 8.3.3 of the HTML5 spec.
    // May generate an exception via the passed ExecState.
    void fillMessagePortArray(JSC::ExecState*, JSC::JSValue, MessagePortArray&);

    // Helper function to convert from JS postMessage arguments to WebCore postMessage arguments.
    template <typename T>
    inline JSC::JSValue handlePostMessage(JSC::ExecState* exec, const JSC::ArgList& args, T* impl)
    {
        PassRefPtr<SerializedScriptValue> message = SerializedScriptValue::create(exec, args.at(0));
        MessagePortArray portArray;
        fillMessagePortArray(exec, args.at(1), portArray);
        if (exec->hadException())
            return JSC::jsUndefined();

        ExceptionCode ec = 0;
        impl->postMessage(message, &portArray, ec);
        setDOMException(exec, ec);
        return JSC::jsUndefined();
    }

}
#endif // JSMessagePortCustom_h
