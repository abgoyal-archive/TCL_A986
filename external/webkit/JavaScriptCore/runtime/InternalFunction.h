

#ifndef InternalFunction_h
#define InternalFunction_h

#include "JSObject.h"
#include "Identifier.h"

namespace JSC {

    class FunctionPrototype;

    class InternalFunction : public JSObject {
    public:
        virtual const ClassInfo* classInfo() const; 
        static JS_EXPORTDATA const ClassInfo info;

        const UString& name(ExecState*);
        const UString displayName(ExecState*);
        const UString calculatedDisplayName(ExecState*);

        static PassRefPtr<Structure> createStructure(JSValue proto) 
        { 
            return Structure::create(proto, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); 
        }

    protected:
        static const unsigned StructureFlags = ImplementsHasInstance | JSObject::StructureFlags;

        InternalFunction(NonNullPassRefPtr<Structure> structure) : JSObject(structure) { }
        InternalFunction(JSGlobalData*, NonNullPassRefPtr<Structure>, const Identifier&);

    private:
        virtual CallType getCallData(CallData&) = 0;
    };

    InternalFunction* asInternalFunction(JSValue);

    inline InternalFunction* asInternalFunction(JSValue value)
    {
        ASSERT(asObject(value)->inherits(&InternalFunction::info));
        return static_cast<InternalFunction*>(asObject(value));
    }

} // namespace JSC

#endif // InternalFunction_h
