

#ifndef _NP_RUNTIME_IMPL_H_
#define _NP_RUNTIME_IMPL_H_

#if ENABLE(NETSCAPE_PLUGIN_API)

#include "npruntime_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void _NPN_ReleaseVariantValue(NPVariant*);
extern NPIdentifier _NPN_GetStringIdentifier(const NPUTF8*);
extern void _NPN_GetStringIdentifiers(const NPUTF8** names, int32_t nameCount, NPIdentifier* identifiers);
extern NPIdentifier _NPN_GetIntIdentifier(int32_t);
extern bool _NPN_IdentifierIsString(NPIdentifier);
extern NPUTF8* _NPN_UTF8FromIdentifier(NPIdentifier);
extern int32_t _NPN_IntFromIdentifier(NPIdentifier);    
extern NPObject* _NPN_CreateObject(NPP, NPClass*);
extern NPObject* _NPN_RetainObject(NPObject*);
extern void _NPN_ReleaseObject(NPObject*);
extern void _NPN_DeallocateObject(NPObject*);
extern bool _NPN_Invoke(NPP, NPObject*, NPIdentifier methodName, const NPVariant* args, uint32_t argCount, NPVariant* result);
extern bool _NPN_InvokeDefault(NPP, NPObject*, const NPVariant* args, uint32_t argCount, NPVariant* result);
extern bool _NPN_Evaluate(NPP, NPObject*, NPString*, NPVariant* result);
extern bool _NPN_GetProperty(NPP, NPObject*, NPIdentifier, NPVariant* result);
extern bool _NPN_SetProperty(NPP, NPObject*, NPIdentifier, const NPVariant*);
extern bool _NPN_RemoveProperty(NPP, NPObject*, NPIdentifier);
extern bool _NPN_HasProperty(NPP, NPObject*, NPIdentifier);
extern bool _NPN_HasMethod(NPP, NPObject*, NPIdentifier);
extern void _NPN_SetException(NPObject*, const NPUTF8*);
extern bool _NPN_Enumerate(NPP, NPObject*, NPIdentifier**, uint32_t* count);
extern bool _NPN_Construct(NPP, NPObject*, const NPVariant* args, uint32_t argCount, NPVariant *result);

#ifdef __cplusplus
}  /* end extern "C" */
#endif

#endif // ENABLE(NETSCAPE_PLUGIN_API)

#endif
