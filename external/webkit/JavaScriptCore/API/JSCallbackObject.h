

#ifndef JSCallbackObject_h
#define JSCallbackObject_h

#include "JSObjectRef.h"
#include "JSValueRef.h"
#include "JSObject.h"

namespace JSC {

template <class Base>
class JSCallbackObject : public Base {
public:
    JSCallbackObject(ExecState*, NonNullPassRefPtr<Structure>, JSClassRef, void* data);
    JSCallbackObject(JSClassRef);
    virtual ~JSCallbackObject();

    void setPrivate(void* data);
    void* getPrivate();

    static const ClassInfo info;

    JSClassRef classRef() const { return m_callbackObjectData->jsClass; }
    bool inherits(JSClassRef) const;

    static PassRefPtr<Structure> createStructure(JSValue proto) 
    { 
        return Structure::create(proto, TypeInfo(ObjectType, StructureFlags), Base::AnonymousSlotCount); 
    }

protected:
    static const unsigned StructureFlags = OverridesGetOwnPropertySlot | ImplementsHasInstance | OverridesHasInstance | OverridesMarkChildren | OverridesGetPropertyNames | Base::StructureFlags;

private:
    virtual UString className() const;

    virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
    virtual bool getOwnPropertySlot(ExecState*, unsigned, PropertySlot&);
    virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);
    
    virtual void put(ExecState*, const Identifier&, JSValue, PutPropertySlot&);

    virtual bool deleteProperty(ExecState*, const Identifier&);
    virtual bool deleteProperty(ExecState*, unsigned);

    virtual bool hasInstance(ExecState* exec, JSValue value, JSValue proto);

    virtual void getOwnPropertyNames(ExecState*, PropertyNameArray&, EnumerationMode mode = ExcludeDontEnumProperties);

    virtual double toNumber(ExecState*) const;
    virtual UString toString(ExecState*) const;

    virtual ConstructType getConstructData(ConstructData&);
    virtual CallType getCallData(CallData&);
    virtual const ClassInfo* classInfo() const { return &info; }

    void init(ExecState*);
 
    static JSCallbackObject* asCallbackObject(JSValue);
 
    static JSValue JSC_HOST_CALL call(ExecState*, JSObject* functionObject, JSValue thisValue, const ArgList&);
    static JSObject* construct(ExecState*, JSObject* constructor, const ArgList&);
   
    static JSValue staticValueGetter(ExecState*, const Identifier&, const PropertySlot&);
    static JSValue staticFunctionGetter(ExecState*, const Identifier&, const PropertySlot&);
    static JSValue callbackGetter(ExecState*, const Identifier&, const PropertySlot&);

    struct JSCallbackObjectData {
        JSCallbackObjectData(void* privateData, JSClassRef jsClass)
            : privateData(privateData)
            , jsClass(jsClass)
        {
            JSClassRetain(jsClass);
        }
        
        ~JSCallbackObjectData()
        {
            JSClassRelease(jsClass);
        }
        
        void* privateData;
        JSClassRef jsClass;
    };
    
    OwnPtr<JSCallbackObjectData> m_callbackObjectData;
};

} // namespace JSC

// include the actual template class implementation
#include "JSCallbackObjectFunctions.h"

#endif // JSCallbackObject_h
