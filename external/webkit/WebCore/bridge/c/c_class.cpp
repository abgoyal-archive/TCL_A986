

#include "config.h"

#if ENABLE(NETSCAPE_PLUGIN_API)

#include "c_class.h"

#include "c_instance.h"
#include "c_runtime.h"
#include "npruntime_impl.h"
#include <runtime/Identifier.h>
#include <runtime/JSLock.h>

namespace JSC { namespace Bindings {

CClass::CClass(NPClass* aClass)
{
    _isa = aClass;
}

CClass::~CClass()
{
    JSLock lock(SilenceAssertionsOnly);

    deleteAllValues(_methods);
    _methods.clear();

    deleteAllValues(_fields);
    _fields.clear();
}

typedef HashMap<NPClass*, CClass*> ClassesByIsAMap;
static ClassesByIsAMap* classesByIsA = 0;

CClass* CClass::classForIsA(NPClass* isa)
{
    if (!classesByIsA)
        classesByIsA = new ClassesByIsAMap;

    CClass* aClass = classesByIsA->get(isa);
    if (!aClass) {
        aClass = new CClass(isa);
        classesByIsA->set(isa, aClass);
    }

    return aClass;
}

MethodList CClass::methodsNamed(const Identifier& identifier, Instance* instance) const
{
    MethodList methodList;

    Method* method = _methods.get(identifier.ustring().rep());
    if (method) {
        methodList.append(method);
        return methodList;
    }

    NPIdentifier ident = _NPN_GetStringIdentifier(identifier.ascii());
    const CInstance* inst = static_cast<const CInstance*>(instance);
    NPObject* obj = inst->getObject();
    if (_isa->hasMethod && _isa->hasMethod(obj, ident)){
        Method* aMethod = new CMethod(ident); // deleted in the CClass destructor
        {
            JSLock lock(SilenceAssertionsOnly);
            _methods.set(identifier.ustring().rep(), aMethod);
        }
        methodList.append(aMethod);
    }
    
    return methodList;
}

Field* CClass::fieldNamed(const Identifier& identifier, Instance* instance) const
{
    Field* aField = _fields.get(identifier.ustring().rep());
    if (aField)
        return aField;
    
    NPIdentifier ident = _NPN_GetStringIdentifier(identifier.ascii());
    const CInstance* inst = static_cast<const CInstance*>(instance);
    NPObject* obj = inst->getObject();
    if (_isa->hasProperty && _isa->hasProperty(obj, ident)){
        aField = new CField(ident); // deleted in the CClass destructor
        {
            JSLock lock(SilenceAssertionsOnly);
            _fields.set(identifier.ustring().rep(), aField);
        }
    }
    return aField;
}

} } // namespace JSC::Bindings

#endif // ENABLE(NETSCAPE_PLUGIN_API)
