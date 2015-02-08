

#ifndef JSNotAnObject_h
#define JSNotAnObject_h

#include "JSObject.h"

namespace JSC {

    class JSNotAnObjectErrorStub : public JSObject {
    public:
        JSNotAnObjectErrorStub(ExecState* exec, bool isNull)
            : JSObject(exec->globalData().notAnObjectErrorStubStructure)
            , m_isNull(isNull)
        {
        }

        bool isNull() const { return m_isNull; }

    private:
        virtual bool isNotAnObjectErrorStub() const { return true; }

        bool m_isNull;
    };
    
    // This unholy class is used to allow us to avoid multiple exception checks
    // in certain SquirrelFish bytecodes -- effectively it just silently consumes
    // any operations performed on the result of a failed toObject call.
    class JSNotAnObject : public JSObject {
    public:
        JSNotAnObject(ExecState* exec, JSNotAnObjectErrorStub* exception)
            : JSObject(exec->globalData().notAnObjectStructure)
            , m_exception(exception)
        {
        }

        static PassRefPtr<Structure> createStructure(JSValue prototype)
        {
            return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
        }

     private:
        
        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | OverridesMarkChildren | OverridesGetPropertyNames | JSObject::StructureFlags;

        // JSValue methods
        virtual JSValue toPrimitive(ExecState*, PreferredPrimitiveType) const;
        virtual bool getPrimitiveNumber(ExecState*, double& number, JSValue&);
        virtual bool toBoolean(ExecState*) const;
        virtual double toNumber(ExecState*) const;
        virtual UString toString(ExecState*) const;
        virtual JSObject* toObject(ExecState*) const;

        // Marking
        virtual void markChildren(MarkStack&);

        // JSObject methods
        virtual bool getOwnPropertySlot(ExecState*, const Identifier& propertyName, PropertySlot&);
        virtual bool getOwnPropertySlot(ExecState*, unsigned propertyName, PropertySlot&);
        virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);

        virtual void put(ExecState*, const Identifier& propertyName, JSValue, PutPropertySlot&);
        virtual void put(ExecState*, unsigned propertyName, JSValue);

        virtual bool deleteProperty(ExecState*, const Identifier& propertyName);
        virtual bool deleteProperty(ExecState*, unsigned propertyName);

        virtual void getOwnPropertyNames(ExecState*, PropertyNameArray&, EnumerationMode mode = ExcludeDontEnumProperties);

        JSNotAnObjectErrorStub* m_exception;
    };

} // namespace JSC

#endif // JSNotAnObject_h
