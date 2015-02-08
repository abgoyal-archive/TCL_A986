

#ifndef JSOptionConstructor_h
#define JSOptionConstructor_h

#include "JSDOMBinding.h"
#include "JSDocument.h"
#include <wtf/RefPtr.h>

namespace WebCore {

    class JSOptionConstructor : public DOMConstructorWithDocument {
    public:
        JSOptionConstructor(JSC::ExecState*, JSDOMGlobalObject*);

        static const JSC::ClassInfo s_info;
    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

} // namespace WebCore

#endif // JSOptionConstructor_h
