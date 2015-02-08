

#ifndef JSWebGLByteArrayConstructor_h
#define JSWebGLByteArrayConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSWebGLByteArrayConstructor : public DOMConstructorObject {
    public:
        JSWebGLByteArrayConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLByteArrayConstructor_h
