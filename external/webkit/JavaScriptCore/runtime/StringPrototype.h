

#ifndef StringPrototype_h
#define StringPrototype_h

#include "StringObject.h"

namespace JSC {

    class ObjectPrototype;

    class StringPrototype : public StringObject {
    public:
        StringPrototype(ExecState*, NonNullPassRefPtr<Structure>);

        virtual bool getOwnPropertySlot(ExecState*, const Identifier& propertyName, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);

        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;
    };

} // namespace JSC

#endif // StringPrototype_h
