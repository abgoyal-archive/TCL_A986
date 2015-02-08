
#ifndef _NP_RUNTIME_H_
#define _NP_RUNTIME_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "npapi.h"



typedef struct NPObject NPObject;
typedef struct NPClass NPClass;

typedef char NPUTF8;
typedef struct _NPString {
    const NPUTF8 *UTF8Characters;
    uint32_t UTF8Length;
} NPString;
  
typedef enum {
    NPVariantType_Void,
    NPVariantType_Null,
    NPVariantType_Bool,
    NPVariantType_Int32,
    NPVariantType_Double,
    NPVariantType_String,
    NPVariantType_Object
} NPVariantType;

typedef struct _NPVariant {
    NPVariantType type;
    union {
        bool boolValue;
        int32_t intValue;
        double doubleValue;
        NPString stringValue;
        NPObject *objectValue;
    } value;
} NPVariant;

void NPN_ReleaseVariantValue (NPVariant *variant);

#define NPVARIANT_IS_VOID(_v)    ((_v).type == NPVariantType_Void)
#define NPVARIANT_IS_NULL(_v)    ((_v).type == NPVariantType_Null)
#define NPVARIANT_IS_BOOLEAN(_v) ((_v).type == NPVariantType_Bool)
#define NPVARIANT_IS_INT32(_v)   ((_v).type == NPVariantType_Int32)
#define NPVARIANT_IS_DOUBLE(_v)  ((_v).type == NPVariantType_Double)
#define NPVARIANT_IS_STRING(_v)  ((_v).type == NPVariantType_String)
#define NPVARIANT_IS_OBJECT(_v)  ((_v).type == NPVariantType_Object)

#define NPVARIANT_TO_BOOLEAN(_v) ((_v).value.boolValue)
#define NPVARIANT_TO_INT32(_v)   ((_v).value.intValue)
#define NPVARIANT_TO_DOUBLE(_v)  ((_v).value.doubleValue)
#define NPVARIANT_TO_STRING(_v)  ((_v).value.stringValue)
#define NPVARIANT_TO_OBJECT(_v)  ((_v).value.objectValue)

#define NP_BEGIN_MACRO  do {
#define NP_END_MACRO    } while (0)

#define VOID_TO_NPVARIANT(_v)                NP_BEGIN_MACRO (_v).type = NPVariantType_Void; (_v).value.objectValue = NULL; NP_END_MACRO
#define NULL_TO_NPVARIANT(_v)                NP_BEGIN_MACRO (_v).type = NPVariantType_Null; (_v).value.objectValue = NULL; NP_END_MACRO
#define BOOLEAN_TO_NPVARIANT(_val, _v)       NP_BEGIN_MACRO (_v).type = NPVariantType_Bool; (_v).value.boolValue = !!(_val); NP_END_MACRO
#define INT32_TO_NPVARIANT(_val, _v)         NP_BEGIN_MACRO (_v).type = NPVariantType_Int32; (_v).value.intValue = _val; NP_END_MACRO
#define DOUBLE_TO_NPVARIANT(_val, _v)        NP_BEGIN_MACRO (_v).type = NPVariantType_Double; (_v).value.doubleValue = _val; NP_END_MACRO
#define STRINGZ_TO_NPVARIANT(_val, _v)       NP_BEGIN_MACRO (_v).type = NPVariantType_String; NPString str = { _val, strlen(_val) }; (_v).value.stringValue = str; NP_END_MACRO
#define STRINGN_TO_NPVARIANT(_val, _len, _v) NP_BEGIN_MACRO (_v).type = NPVariantType_String; NPString str = { _val, _len }; (_v).value.stringValue = str; NP_END_MACRO
#define OBJECT_TO_NPVARIANT(_val, _v)        NP_BEGIN_MACRO (_v).type = NPVariantType_Object; (_v).value.objectValue = _val; NP_END_MACRO


typedef void *NPIdentifier;

NPIdentifier NPN_GetStringIdentifier(const NPUTF8 *name);
void NPN_GetStringIdentifiers(const NPUTF8 **names, int32_t nameCount, NPIdentifier *identifiers);
NPIdentifier NPN_GetIntIdentifier(int32_t intid);
bool NPN_IdentifierIsString(NPIdentifier identifier);

NPUTF8 *NPN_UTF8FromIdentifier(NPIdentifier identifier);

int32_t NPN_IntFromIdentifier(NPIdentifier identifier);

typedef NPObject *(*NPAllocateFunctionPtr)(NPP npp, NPClass *aClass);
typedef void (*NPDeallocateFunctionPtr)(NPObject *obj);
typedef void (*NPInvalidateFunctionPtr)(NPObject *obj);
typedef bool (*NPHasMethodFunctionPtr)(NPObject *obj, NPIdentifier name);
typedef bool (*NPInvokeFunctionPtr)(NPObject *obj, NPIdentifier name, const NPVariant *args, uint32_t argCount, NPVariant *result);
typedef bool (*NPInvokeDefaultFunctionPtr)(NPObject *npobj, const NPVariant *args, uint32_t argCount, NPVariant *result);
typedef bool (*NPHasPropertyFunctionPtr)(NPObject *obj, NPIdentifier name);
typedef bool (*NPGetPropertyFunctionPtr)(NPObject *obj, NPIdentifier name, NPVariant *result);
typedef bool (*NPSetPropertyFunctionPtr)(NPObject *obj, NPIdentifier name, const NPVariant *value);
typedef bool (*NPRemovePropertyFunctionPtr)(NPObject *npobj, NPIdentifier name);
typedef bool (*NPEnumerationFunctionPtr)(NPObject *npobj, NPIdentifier **value, uint32_t *count);
typedef bool (*NPConstructFunctionPtr)(NPObject *npobj, const NPVariant *args, uint32_t argCount, NPVariant *result);

struct NPClass
{
    uint32_t structVersion;
    NPAllocateFunctionPtr allocate;
    NPDeallocateFunctionPtr deallocate;
    NPInvalidateFunctionPtr invalidate;
    NPHasMethodFunctionPtr hasMethod;
    NPInvokeFunctionPtr invoke;
    NPInvokeDefaultFunctionPtr invokeDefault;
    NPHasPropertyFunctionPtr hasProperty;
    NPGetPropertyFunctionPtr getProperty;
    NPSetPropertyFunctionPtr setProperty;
    NPRemovePropertyFunctionPtr removeProperty;
    NPEnumerationFunctionPtr enumerate;
    NPConstructFunctionPtr construct;
};

#define NP_CLASS_STRUCT_VERSION      3
#define NP_CLASS_STRUCT_VERSION_ENUM 2
#define NP_CLASS_STRUCT_VERSION_CTOR 3

#define NP_CLASS_STRUCT_VERSION_HAS_ENUM(npclass)   \
    ((npclass)->structVersion >= NP_CLASS_STRUCT_VERSION_ENUM)
#define NP_CLASS_STRUCT_VERSION_HAS_CTOR(npclass)   \
    ((npclass)->structVersion >= NP_CLASS_STRUCT_VERSION_CTOR)

struct NPObject {
    NPClass *_class;
    uint32_t referenceCount;
    // Additional space may be allocated here by types of NPObjects
};

NPObject *NPN_CreateObject(NPP npp, NPClass *aClass);

NPObject *NPN_RetainObject (NPObject *obj);

void NPN_ReleaseObject (NPObject *obj);

bool NPN_Invoke(NPP npp, NPObject *npobj, NPIdentifier methodName, const NPVariant *args, uint32_t argCount, NPVariant *result);
bool NPN_InvokeDefault(NPP npp, NPObject *npobj, const NPVariant *args, uint32_t argCount, NPVariant *result);
bool NPN_Evaluate(NPP npp, NPObject *npobj, NPString *script, NPVariant *result);
bool NPN_GetProperty(NPP npp, NPObject *npobj, NPIdentifier propertyName, NPVariant *result);
bool NPN_SetProperty(NPP npp, NPObject *npobj, NPIdentifier propertyName, const NPVariant *value);
bool NPN_RemoveProperty(NPP npp, NPObject *npobj, NPIdentifier propertyName);
bool NPN_HasProperty(NPP npp, NPObject *npobj, NPIdentifier propertyName);
bool NPN_HasMethod(NPP npp, NPObject *npobj, NPIdentifier methodName);
bool NPN_Enumerate(NPP npp, NPObject *npobj, NPIdentifier **identifier, uint32_t *count);
bool NPN_Construct(NPP npp, NPObject *npobj, const NPVariant *args, uint32_t argCount, NPVariant *result);

void NPN_SetException (NPObject *obj, const NPUTF8 *message);

#ifdef __cplusplus
}
#endif

#endif
