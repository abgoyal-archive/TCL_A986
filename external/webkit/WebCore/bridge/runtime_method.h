

#ifndef RUNTIME_FUNCTION_H_
#define RUNTIME_FUNCTION_H_

#include "Bridge.h"
#include <runtime/InternalFunction.h>
#include <runtime/JSGlobalObject.h>
#include <wtf/OwnPtr.h>

namespace JSC {

class RuntimeMethod : public InternalFunction {
public:
    RuntimeMethod(ExecState*, const Identifier& name, Bindings::MethodList&);
    Bindings::MethodList* methods() const { return _methodList.get(); }

    static const ClassInfo s_info;

    static FunctionPrototype* createPrototype(ExecState*, JSGlobalObject* globalObject)
    {
        return globalObject->functionPrototype();
    }

    static PassRefPtr<Structure> createStructure(JSValue prototype)
    {
        return Structure::create(prototype, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount);
    }

private:
    static const unsigned StructureFlags = OverridesGetOwnPropertySlot | ImplementsHasInstance | OverridesMarkChildren | InternalFunction::StructureFlags;
    static JSValue lengthGetter(ExecState*, const Identifier&, const PropertySlot&);
    virtual bool getOwnPropertySlot(ExecState*, const Identifier&, PropertySlot&);
    virtual bool getOwnPropertyDescriptor(ExecState*, const Identifier&, PropertyDescriptor&);
    virtual CallType getCallData(CallData&);

    OwnPtr<Bindings::MethodList> _methodList;
};

} // namespace JSC

#endif
