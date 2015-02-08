

#ifndef DateInstance_h
#define DateInstance_h

#include "JSWrapperObject.h"

namespace WTF {
    struct GregorianDateTime;
}

namespace JSC {

    class DateInstance : public JSWrapperObject {
    public:
        DateInstance(ExecState*, double);
        DateInstance(ExecState*, NonNullPassRefPtr<Structure>, double);
        explicit DateInstance(ExecState*, NonNullPassRefPtr<Structure>);

        double internalNumber() const { return internalValue().uncheckedGetNumber(); }

        static JS_EXPORTDATA const ClassInfo info;

        const GregorianDateTime* gregorianDateTime(ExecState* exec) const
        {
            if (m_data && m_data->m_gregorianDateTimeCachedForMS == internalNumber())
                return &m_data->m_cachedGregorianDateTime;
            return calculateGregorianDateTime(exec);
        }
        
        const GregorianDateTime* gregorianDateTimeUTC(ExecState* exec) const
        {
            if (m_data && m_data->m_gregorianDateTimeUTCCachedForMS == internalNumber())
                return &m_data->m_cachedGregorianDateTimeUTC;
            return calculateGregorianDateTimeUTC(exec);
        }

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

    protected:
        static const unsigned StructureFlags = OverridesMarkChildren | JSWrapperObject::StructureFlags;

    private:
        const GregorianDateTime* calculateGregorianDateTime(ExecState*) const;
        const GregorianDateTime* calculateGregorianDateTimeUTC(ExecState*) const;
        virtual const ClassInfo* classInfo() const { return &info; }

        mutable RefPtr<DateInstanceData> m_data;
    };

    DateInstance* asDateInstance(JSValue);

    inline DateInstance* asDateInstance(JSValue value)
    {
        ASSERT(asObject(value)->inherits(&DateInstance::info));
        return static_cast<DateInstance*>(asObject(value));
    }

} // namespace JSC

#endif // DateInstance_h
