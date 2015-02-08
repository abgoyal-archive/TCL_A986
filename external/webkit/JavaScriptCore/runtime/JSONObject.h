

#ifndef JSONObject_h
#define JSONObject_h

#include "JSObject.h"

namespace JSC {

    class Stringifier;

    class JSONObject : public JSObject {
    public:
        JSONObject(NonNullPassRefPtr<Structure> structure)
            : JSObject(structure)
        {
        }

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

        static void markStringifiers(MarkStack&, Stringifier*);

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | JSObject::StructureFlags;

    private:
        virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);

        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;
    };

} // namespace JSC

#endif // JSONObject_h
