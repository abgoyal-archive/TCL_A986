

#ifndef JSWorkerConstructor_h
#define JSWorkerConstructor_h

#if ENABLE(WORKERS)

#include "JSDOMBinding.h"

namespace WebCore {

    class JSWorkerConstructor : public DOMConstructorObject {
    public:
        JSWorkerConstructor(JSC::ExecState*, JSDOMGlobalObject*);

        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);

        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // JSWorkerConstructor_h
