

#ifndef JSWebGLFloatArrayConstructor_h
#define JSWebGLFloatArrayConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSWebGLFloatArrayConstructor : public DOMConstructorObject {
    public:
        JSWebGLFloatArrayConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLFloatArrayConstructor_h
