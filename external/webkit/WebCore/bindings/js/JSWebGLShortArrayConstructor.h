

#ifndef JSWebGLShortArrayConstructor_h
#define JSWebGLShortArrayConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSWebGLShortArrayConstructor : public DOMConstructorObject {
    public:
        JSWebGLShortArrayConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

}

#endif // JSWebGLShortArrayConstructor_h
