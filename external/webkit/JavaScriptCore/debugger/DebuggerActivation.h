

#ifndef DebuggerActivation_h
#define DebuggerActivation_h

#include "JSObject.h"

namespace JSC {

    class JSActivation;

    class DebuggerActivation : public JSObject {
    public:
        DebuggerActivation(JSObject*);

        virtual void markChildren(MarkStack&);
        virtual UString className() const;
        virtual bool getOwnPropertySlot(ExecState*, const Identifier& propertyName, PropertySlot&);
        virtual void put(ExecState*, const Identifier& propertyName, JSValue, PutPropertySlot&);
        virtual void putWithAttributes(ExecState*, const Identifier& propertyName, JSValue, unsigned attributes);
        virtual bool deleteProperty(ExecState*, const Identifier& propertyName);
        virtual void getOwnPropertyNames(ExecState*, PropertyNameArray&, EnumerationMode mode = ExcludeDontEnumProperties);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);
        virtual void defineGetter(ExecState*, const Identifier& propertyName, JSObject* getterFunction, unsigned attributes);
        virtual void defineSetter(ExecState*, const Identifier& propertyName, JSObject* setterFunction, unsigned attributes);
        virtual JSValue lookupGetter(ExecState*, const Identifier& propertyName);
        virtual JSValue lookupSetter(ExecState*, const Identifier& propertyName);

        static PassRefPtr<Structure> createStructure(JSValue prototype) 
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); 
        }

    protected:
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | OverridesMarkChildren | JSObject::StructureFlags;

    private:
        JSActivation* m_activation;
    };

} // namespace JSC

#endif // DebuggerActivation_h
