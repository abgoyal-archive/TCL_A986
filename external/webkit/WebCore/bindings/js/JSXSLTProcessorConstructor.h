

#ifndef JSXSLTProcessorConstructor_h
#define JSXSLTProcessorConstructor_h

#if ENABLE(XSLT)

#include "JSDOMBinding.h"

namespace WebCore {

    class JSXSLTProcessorConstructor : public DOMConstructorObject {
    public:
        JSXSLTProcessorConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

} // namespace WebCore

#endif // ENABLE(XSLT)

#endif // JSXSLTProcessorConstructor_h
