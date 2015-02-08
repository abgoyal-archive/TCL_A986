

#ifndef RUNTIME_ARRAY_H_
#define RUNTIME_ARRAY_H_

#include "Bridge.h"
#include <runtime/JSGlobalObject.h>

namespace JSC {
    
class RuntimeArray : public JSObject {
public:
    RuntimeArray(ExecState*, Bindings::Array*);

    virtual void getOwnPropertyNames(ExecState*, PropertyNameArray&, EnumerationMode mode = ExcludeDontEnumProperties);
    virtual bool getOwnPropertySlot(ExecState *, const Identifier&, PropertySlot&);
    virtual bool getOwnPropertySlot(ExecState *, unsigned, PropertySlot&);
    virtual bool getOwnPropertyDescriptor(ExecState *, const Identifier&, PropertyDescriptor&);
    virtual void put(ExecState*, const Identifier& propertyName, JSValue, PutPropertySlot&);
    virtual void put(ExecState*, unsigned propertyName, JSValue);
    
    virtual bool deleteProperty(ExecState *exec, const Identifier &propertyName);
    virtual bool deleteProperty(ExecState *exec, unsigned propertyName);
    
    virtual const ClassInfo *classInfo() const { return &s_info; }
    
    unsigned getLength() const { return getConcreteArray()->getLength(); }
    
    Bindings::Array *getConcreteArray() const { return _array.get(); }

    static const ClassInfo s_info;

    static ArrayPrototype* createPrototype(ExecState*, JSGlobalObject* globalObject)
    {
        return globalObject->arrayPrototype();
    }

    static PassRefPtr<Structure> createStructure(JSValue prototype)
    {
        return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
    }

private:
    static const unsigned StructureFlags = OverridesGetOwnPropertySlot | OverridesGetPropertyNames | JSObject::StructureFlags;
    static JSValue lengthGetter(ExecState*, const Identifier&, const PropertySlot&);
    static JSValue indexGetter(ExecState*, const Identifier&, const PropertySlot&);

    OwnPtr<Bindings::Array> _array;
};
    
} // namespace JSC

#endif // RUNTIME_ARRAY_H_
