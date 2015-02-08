

#ifndef JSCallbackFunction_h
#define JSCallbackFunction_h

#include "InternalFunction.h"
#include "JSObjectRef.h"

namespace JSC {

class JSCallbackFunction : public InternalFunction {
public:
    JSCallbackFunction(ExecState*, JSObjectCallAsFunctionCallback, const Identifier& name);

    static const ClassInfo info;
    
    // InternalFunction mish-mashes constructor and function behavior -- we should 
    // refactor the code so this override isn't necessary
    static PassRefPtr<Structure> createStructure(JSValue proto) 
    { 
        return Structure::create(proto, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); 
    }

private:
    virtual CallType getCallData(CallData&);
    virtual const ClassInfo* classInfo() const { return &info; }

    static JSValue JSC_HOST_CALL call(ExecState*, JSObject*, JSValue, const ArgList&);

    JSObjectCallAsFunctionCallback m_callback;
};

} // namespace JSC

#endif // JSCallbackFunction_h
