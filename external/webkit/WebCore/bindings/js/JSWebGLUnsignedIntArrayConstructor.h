

#ifndef JSWebGLUnsignedIntArrayConstructor_h
#define JSWebGLUnsignedIntArrayConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSWebGLUnsignedIntArrayConstructor : public DOMConstructorObject {
    public:
        JSWebGLUnsignedIntArrayConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLUnsignedIntArrayConstructor_h
