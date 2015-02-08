

#ifndef JSSharedWorkerConstructor_h
#define JSSharedWorkerConstructor_h

#if ENABLE(SHARED_WORKERS)

#include "JSDOMBinding.h"

namespace WebCore {

    class JSSharedWorkerConstructor : public DOMConstructorObject {
    public:
        JSSharedWorkerConstructor(JSC::ExecState*, JSDOMGlobalObject*);

        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);

        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)

#endif // JSSharedWorkerConstructor_h
