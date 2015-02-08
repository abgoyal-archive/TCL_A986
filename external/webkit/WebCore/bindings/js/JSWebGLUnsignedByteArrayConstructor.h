

#ifndef JSWebGLUnsignedByteArrayConstructor_h
#define JSWebGLUnsignedByteArrayConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSWebGLUnsignedByteArrayConstructor : public DOMConstructorObject {
    public:
        JSWebGLUnsignedByteArrayConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLUnsignedByteArrayConstructor_h
