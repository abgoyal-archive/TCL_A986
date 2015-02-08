

#ifndef JSWebGLArrayBufferConstructor_h
#define JSWebGLArrayBufferConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"
#include "JSWebGLArrayBuffer.h"
#include <runtime/Error.h>

namespace WebCore {

    class WebGLArray;

    // Template function used by CanvasXXXArrayConstructors
    template<class C, typename T>
    PassRefPtr<WebGLArray> construct(JSC::ExecState* exec, const JSC::ArgList& args)
    {
        // There are 3 constructors:
        //
        //  1) (in int size)
        //  2) (in WebGLArrayBuffer buffer, [Optional] in int offset, [Optional] in unsigned int length)
        //  3) (in sequence<T>) - This ends up being a JS "array-like" object
        //
        RefPtr<C> arrayObject;
        
        // For the 0 args case, just create an object without a buffer 
        if (args.size() < 1)
            return C::create(0, 0, 0);
        
        if (args.at(0).isObject()) {
            RefPtr<WebGLArrayBuffer> buffer = toWebGLArrayBuffer(args.at(0));
            if (buffer) {
                int offset = (args.size() > 1) ? args.at(1).toInt32(exec) : 0;
                unsigned int length = (args.size() > 2) ? static_cast<unsigned int>(args.at(2).toInt32(exec)) : 0;
                return C::create(buffer, offset, length);
            }
            
            JSC::JSObject* array = asObject(args.at(0));
            int length = array->get(exec, JSC::Identifier(exec, "length")).toInt32(exec);
            void* tempValues;
            if (!tryFastMalloc(length * sizeof(T)).getValue(tempValues)) {
                throwError(exec, JSC::GeneralError);
                return 0;
            }
            
            OwnFastMallocPtr<T> values(static_cast<T*>(tempValues));
            for (int i = 0; i < length; ++i) {
                JSC::JSValue v = array->get(exec, i);
                if (exec->hadException())
                    return 0;
                values.get()[i] = static_cast<T>(v.toNumber(exec));
            }
            
            return C::create(values.get(), length);
        }
        
        unsigned size = static_cast<unsigned>(args.at(0).toInt32(exec));
        return C::create(size);
    }

    class JSWebGLArrayBufferConstructor : public DOMConstructorObject {
    public:
        JSWebGLArrayBufferConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLArrayBufferConstructor_h
