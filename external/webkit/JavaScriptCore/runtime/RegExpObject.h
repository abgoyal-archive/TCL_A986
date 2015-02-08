

#ifndef RegExpObject_h
#define RegExpObject_h

#include "JSObject.h"
#include "RegExp.h"

namespace JSC {

    class RegExpObject : public JSObject {
    public:
        RegExpObject(NonNullPassRefPtr<Structure>, NonNullPassRefPtr<RegExp>);
        virtual ~RegExpObject();

        void setRegExp(PassRefPtr<RegExp> r) { d->regExp = r; }
        RegExp* regExp() const { return d->regExp.get(); }

        void setLastIndex(double lastIndex) { d->lastIndex = lastIndex; }
        double lastIndex() const { return d->lastIndex; }

        JSValue test(ExecState*, const ArgList&);
        JSValue exec(ExecState*, const ArgList&);

        virtual bool getOwnPropertySlot(ExecState*, const Identifier& propertyName, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);
        virtual void put(ExecState*, const Identifier& propertyName, JSValue, PutPropertySlot&);

        virtual const ClassInfo* classInfo() const { return &info; }
        static const ClassInfo info;

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | JSObject::StructureFlags;

    private:
        bool match(ExecState*, const ArgList&);

        virtual CallType getCallData(CallData&);

        struct RegExpObjectData : FastAllocBase {
            RegExpObjectData(NonNullPassRefPtr<RegExp> regExp, double lastIndex)
                : regExp(regExp)
                , lastIndex(lastIndex)
            {
            }

            RefPtr<RegExp> regExp;
            double lastIndex;
        };

        OwnPtr<RegExpObjectData> d;
    };

    RegExpObject* asRegExpObject(JSValue);

    inline RegExpObject* asRegExpObject(JSValue value)
    {
        ASSERT(asObject(value)->inherits(&RegExpObject::info));
        return static_cast<RegExpObject*>(asObject(value));
    }

} // namespace JSC

#endif // RegExpObject_h
