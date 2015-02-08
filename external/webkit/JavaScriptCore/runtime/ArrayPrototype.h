

#ifndef ArrayPrototype_h
#define ArrayPrototype_h

#include "JSArray.h"
#include "Lookup.h"

namespace JSC {

    class ArrayPrototype : public JSArray {
    public:
        explicit ArrayPrototype(NonNullPassRefPtr<Structure>);

        bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);

        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;
    };

} // namespace JSC

#endif // ArrayPrototype_h
