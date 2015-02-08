

#ifndef BINDINGS_C_CLASS_H_
#define BINDINGS_C_CLASS_H_

#if ENABLE(NETSCAPE_PLUGIN_API)

#include "Bridge.h"
#include "npruntime_internal.h"
#include <wtf/HashMap.h>

namespace JSC {
namespace Bindings {

class CClass : public Class {
protected:
    CClass(NPClass*); // Use classForIsA to create a CClass.
    
public:
    static CClass* classForIsA(NPClass*);
    virtual ~CClass();

    virtual MethodList methodsNamed(const Identifier&, Instance*) const;
    virtual Field* fieldNamed(const Identifier&, Instance*) const;

private:
    NPClass* _isa;
    mutable MethodMap _methods;
    mutable FieldMap _fields;
};

} // namespace Bindings
} // namespace JSC

#endif // ENABLE(NETSCAPE_PLUGIN_API)

#endif
