

#ifndef ErrorInstance_h
#define ErrorInstance_h

#include "JSObject.h"

namespace JSC {

    class ErrorInstance : public JSObject {
    public:
        explicit ErrorInstance(NonNullPassRefPtr<Structure>);

        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;
    };

} // namespace JSC

#endif // ErrorInstance_h
