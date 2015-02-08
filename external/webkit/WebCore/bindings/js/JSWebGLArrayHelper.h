

#ifndef JSWebGLArrayHelper_h
#define JSWebGLArrayHelper_h

#include "ExceptionCode.h"
#include "JSDOMBinding.h"
#include <interpreter/CallFrame.h>
#include <runtime/ArgList.h>
#include <runtime/Error.h>
#include <runtime/JSObject.h>
#include <runtime/JSValue.h>

namespace WebCore {

template <class T>
JSC::JSValue setWebGLArrayFromArray(JSC::ExecState* exec, T* webGLArray, JSC::ArgList const& args)
{
    if (args.at(0).isObject()) {
        // void set(in sequence<long> array, [Optional] in unsigned long offset);
        JSC::JSObject* array = JSC::asObject(args.at(0));
        unsigned offset = 0;
        if (args.size() == 2)
            offset = args.at(1).toInt32(exec);
        int length = array->get(exec, JSC::Identifier(exec, "length")).toInt32(exec);
        if (offset + length > webGLArray->length())
            setDOMException(exec, INDEX_SIZE_ERR);
        else {
            for (int i = 0; i < length; i++) {
                JSC::JSValue v = array->get(exec, i);
                if (exec->hadException())
                    return JSC::jsUndefined();
                webGLArray->set(i + offset, v.toNumber(exec));
            }
        }

        return JSC::jsUndefined();
    }

    return JSC::throwError(exec, JSC::SyntaxError);
}

}

#endif // JSWebGLArrayHelper_h
