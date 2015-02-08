

#include "config.h"

#if ENABLE(NETSCAPE_PLUGIN_API)

#include "c_runtime.h"

#include "c_instance.h"
#include "c_utility.h"
#include "npruntime_impl.h"
#include <runtime/JSLock.h>

namespace JSC {
namespace Bindings {

JSValue CField::valueFromInstance(ExecState* exec, const Instance* inst) const
{
    const CInstance* instance = static_cast<const CInstance*>(inst);
    NPObject* obj = instance->getObject();
    if (obj->_class->getProperty) {
        NPVariant property;
        VOID_TO_NPVARIANT(property);

        bool result;
        {
            JSLock::DropAllLocks dropAllLocks(SilenceAssertionsOnly);
            result = obj->_class->getProperty(obj, _fieldIdentifier, &property);
            CInstance::moveGlobalExceptionToExecState(exec);
        }
        if (result) {
            JSValue result = convertNPVariantToValue(exec, &property, instance->rootObject());
            _NPN_ReleaseVariantValue(&property);
            return result;
        }
    }
    return jsUndefined();
}

void CField::setValueToInstance(ExecState *exec, const Instance *inst, JSValue aValue) const
{
    const CInstance* instance = static_cast<const CInstance*>(inst);
    NPObject* obj = instance->getObject();
    if (obj->_class->setProperty) {
        NPVariant variant;
        convertValueToNPVariant(exec, aValue, &variant);

        {
            JSLock::DropAllLocks dropAllLocks(SilenceAssertionsOnly);
            obj->_class->setProperty(obj, _fieldIdentifier, &variant);
            CInstance::moveGlobalExceptionToExecState(exec);
        }

        _NPN_ReleaseVariantValue(&variant);
    }
}

} }

#endif // ENABLE(NETSCAPE_PLUGIN_API)
