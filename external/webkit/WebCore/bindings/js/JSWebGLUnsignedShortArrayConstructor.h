

#ifndef JSWebGLUnsignedShortArrayConstructor_h
#define JSWebGLUnsignedShortArrayConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSWebGLUnsignedShortArrayConstructor : public DOMConstructorObject {
    public:
        JSWebGLUnsignedShortArrayConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLUnsignedShortArrayConstructor_h
