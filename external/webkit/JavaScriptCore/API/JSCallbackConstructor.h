

#ifndef JSCallbackConstructor_h
#define JSCallbackConstructor_h

#include "JSObjectRef.h"
#include <runtime/JSObject.h>

namespace JSC {

class JSCallbackConstructor : public JSObject {
public:
    JSCallbackConstructor(NonNullPassRefPtr<Structure>, JSClassRef, JSObjectCallAsConstructorCallback);
    virtual ~JSCallbackConstructor();
    JSClassRef classRef() const { return m_class; }
    JSObjectCallAsConstructorCallback callback() const { return m_callback; }
    static const ClassInfo info;
    
    static PassRefPtr<Structure> createStructure(JSValue proto) 
    { 
        return Structure::create(proto, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); 
    }

protected:
    static const unsigned StructureFlags = ImplementsHasInstance | JSObject::StructureFlags;

private:
    virtual ConstructType getConstructData(ConstructData&);
    virtual const ClassInfo* classInfo() const { return &info; }

    JSClassRef m_class;
    JSObjectCallAsConstructorCallback m_callback;
};

} // namespace JSC

#endif // JSCallbackConstructor_h
