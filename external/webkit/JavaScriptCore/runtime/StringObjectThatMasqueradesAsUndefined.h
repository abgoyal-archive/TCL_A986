

#ifndef StringObjectThatMasqueradesAsUndefined_h
#define StringObjectThatMasqueradesAsUndefined_h

#include "JSGlobalObject.h"
#include "StringObject.h"
#include "UString.h"

namespace JSC {

    // WebCore uses this to make style.filter undetectable
    class StringObjectThatMasqueradesAsUndefined : public StringObject {
    public:
        static StringObjectThatMasqueradesAsUndefined* create(ExecState* exec, const UString& string)
        {
            return new (exec) StringObjectThatMasqueradesAsUndefined(exec,
                createStructure(exec->lexicalGlobalObject()->stringPrototype()), string);
        }

    private:
        StringObjectThatMasqueradesAsUndefined(ExecState* exec, NonNullPassRefPtr<Structure> structure, const UString& string)
            : StringObject(exec, structure, string)
        {
        }

        static PassRefPtr<Structure> createStructure(JSValue proto) 
        { 
            return Structure::create(proto, TypeInfo(ObjectType, StructureFlags), AnonymousSlotCount); 
        }

        static const unsigned StructureFlags = OverridesGetOwnPropertySlot | MasqueradesAsUndefined | OverridesGetPropertyNames | StringObject::StructureFlags;

        virtual bool toBoolean(ExecState*) const { return false; }
    };
 
} // namespace JSC

#endif // StringObjectThatMasqueradesAsUndefined_h
