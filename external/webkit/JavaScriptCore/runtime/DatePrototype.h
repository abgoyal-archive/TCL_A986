

#ifndef DatePrototype_h
#define DatePrototype_h

#include "DateInstance.h"

namespace JSC {

    class ObjectPrototype;

    class DatePrototype : public DateInstance {
    public:
        DatePrototype(ExecState*, NonNullPassRefPtr<Structure>);

        virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);

        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | DateInstance::StructureFlags;

    };

} // namespace JSC

#endif // DatePrototype_h
