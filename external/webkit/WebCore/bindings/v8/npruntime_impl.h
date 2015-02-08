

#ifndef npruntime_impl_h
#define npruntime_impl_h

#if PLATFORM(CHROMIUM)
#include <bindings/npruntime.h>
#else
#include "npruntime.h"
#endif

// This file exists to support WebCore, which expects to be able to call upon
// portions of the NPRuntime implementation.

#ifdef __cplusplus
extern "C" {
#endif

NPIdentifier _NPN_GetStringIdentifier(const NPUTF8* name);
void _NPN_GetStringIdentifiers(const NPUTF8** names, int32_t nameCount, NPIdentifier*);
NPIdentifier _NPN_GetIntIdentifier(int32_t intId);
bool _NPN_IdentifierIsString(NPIdentifier);
NPUTF8 *_NPN_UTF8FromIdentifier(NPIdentifier);
int32_t _NPN_IntFromIdentifier(NPIdentifier);
void _NPN_ReleaseVariantValue(NPVariant*);
NPObject *_NPN_CreateObject(NPP, NPClass*);
NPObject* _NPN_RetainObject(NPObject*);
void _NPN_ReleaseObject(NPObject*);
bool _NPN_Invoke(NPP, NPObject*, NPIdentifier methodName, const NPVariant* arguments, uint32_t argumentCount, NPVariant* result);
bool _NPN_InvokeDefault(NPP, NPObject*, const NPVariant* arguments, uint32_t argumentCount, NPVariant* result);
bool _NPN_Evaluate(NPP, NPObject*, NPString* npScript, NPVariant* result);
bool _NPN_EvaluateHelper(NPP, bool popupsAllowed, NPObject*, NPString* npScript, NPVariant* result);
bool _NPN_GetProperty(NPP, NPObject*, NPIdentifier propertyName, NPVariant* result);
bool _NPN_SetProperty(NPP, NPObject*, NPIdentifier propertyName, const NPVariant* value);
bool _NPN_RemoveProperty(NPP, NPObject*, NPIdentifier propertyName);
bool _NPN_HasProperty(NPP, NPObject*, NPIdentifier propertyName);
bool _NPN_HasMethod(NPP, NPObject*, NPIdentifier methodName);
void _NPN_SetException(NPObject*, const NPUTF8 *message);
bool _NPN_Enumerate(NPP, NPObject*, NPIdentifier**, uint32_t* count);
bool _NPN_Construct(NPP, NPObject*, const NPVariant* arguments, uint32_t argumentCount, NPVariant* result);

#ifdef __cplusplus
}  /* end extern "C" */
#endif

#endif
