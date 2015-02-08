

#ifndef NumberObject_h
#define NumberObject_h

#include "JSWrapperObject.h"

namespace JSC {

    class NumberObject : public JSWrapperObject {
    public:
        explicit NumberObject(NonNullPassRefPtr<Structure>);

        static const ClassInfo info;

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

    protected:
#if USE(JSVALUE32)
        static const unsigned StructureFlags = OverridesMarkChildren | JSWrapperObject::StructureFlags;
#else
        static const unsigned StructureFlags = JSWrapperObject::StructureFlags;
#endif

    private:
        virtual const ClassInfo* classInfo() const { return &info; }

        virtual JSValue getJSNumber();
    };

    NumberObject* constructNumber(ExecState*, JSValue);

} // namespace JSC

#endif // NumberObject_h
