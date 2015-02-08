

#ifndef JSWrapperObject_h
#define JSWrapperObject_h

#include "JSObject.h"

namespace JSC {

    // This class is used as a base for classes such as String,
    // Number, Boolean and Date which are wrappers for primitive types.
    class JSWrapperObject : public JSObject {
    protected:
        explicit JSWrapperObject(NonNullPassRefPtr<Structure>);

    public:
        JSValue internalValue() const { return m_internalValue; }
        void setInternalValue(JSValue);

        static PassRefPtr<Structure> createStructure(JSValue prototype) 
        { 
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

    protected:
        static const unsigned AnonymousSlotCount = 1 + JSObject::AnonymousSlotCount;

    private:
        virtual void markChildren(MarkStack&);
        
        JSValue m_internalValue;
    };

    inline JSWrapperObject::JSWrapperObject(NonNullPassRefPtr<Structure> structure)
        : JSObject(structure)
    {
        putAnonymousValue(0, jsNull());
    }

    inline void JSWrapperObject::setInternalValue(JSValue value)
    {
        ASSERT(value);
        ASSERT(!value.isObject());
        m_internalValue = value;
        putAnonymousValue(0, value);
    }

} // namespace JSC

#endif // JSWrapperObject_h
