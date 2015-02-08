

#ifndef JSWebGLIntArrayConstructor_h
#define JSWebGLIntArrayConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSWebGLIntArrayConstructor : public DOMConstructorObject {
    public:
        JSWebGLIntArrayConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLIntArrayConstructor_h
