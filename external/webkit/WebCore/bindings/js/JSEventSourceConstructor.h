

#ifndef JSEventSourceConstructor_h
#define JSEventSourceConstructor_h

#if ENABLE(EVENTSOURCE)

#include "JSDOMBinding.h"

namespace WebCore {

    class JSEventSourceConstructor : public DOMConstructorObject {
    public:
        JSEventSourceConstructor(JSC::ExecState*, JSDOMGlobalObject*);
        static const JSC::ClassInfo s_info;

    private:
        virtual JSC::ConstructType getConstructData(JSC::ConstructData&);
        virtual const JSC::ClassInfo* classInfo() const { return &s_info; }
    };

} // namespace WebCore

#endif // ENABLE(EVENTSOURCE)

#endif // JSEventSourceConstructor_h
