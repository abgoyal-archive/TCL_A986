

#ifndef JSImageConstructor_h
#define JSImageConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"

namespace WebCore {

    class JSImageConstructor : public DOMConstructorWithDocument {
    public:
        JSImageConstructor(JSC::ExecState*, JSDOMGlobalObject*);

        static const JSC::ClassInfo s_info;
    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

} // namespace WebCore

#endif // JSImageConstructor_h
