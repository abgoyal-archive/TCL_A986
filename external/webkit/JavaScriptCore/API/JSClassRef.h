

#ifndef JSClassRef_h
#define JSClassRef_h

#include "JSObjectRef.h"

#include <runtime/JSObject.h>
#include <runtime/Protect.h>
#include <runtime/UString.h>
#include <runtime/WeakGCPtr.h>
#include <wtf/HashMap.h>
#include <wtf/RefCounted.h>

struct StaticValueEntry : FastAllocBase {
    StaticValueEntry(JSObjectGetPropertyCallback _getProperty, JSObjectSetPropertyCallback _setProperty, JSPropertyAttributes _attributes)
        : getProperty(_getProperty), setProperty(_setProperty), attributes(_attributes)
    {
    }
    
    JSObjectGetPropertyCallback getProperty;
    JSObjectSetPropertyCallback setProperty;
    JSPropertyAttributes attributes;
};

struct StaticFunctionEntry : FastAllocBase {
    StaticFunctionEntry(JSObjectCallAsFunctionCallback _callAsFunction, JSPropertyAttributes _attributes)
        : callAsFunction(_callAsFunction), attributes(_attributes)
    {
    }

    JSObjectCallAsFunctionCallback callAsFunction;
    JSPropertyAttributes attributes;
};

typedef HashMap<RefPtr<JSC::UString::Rep>, StaticValueEntry*> OpaqueJSClassStaticValuesTable;
typedef HashMap<RefPtr<JSC::UString::Rep>, StaticFunctionEntry*> OpaqueJSClassStaticFunctionsTable;

struct OpaqueJSClass;

// An OpaqueJSClass (JSClass) is created without a context, so it can be used with any context, even across context groups.
// This structure holds data members that vary across context groups.
struct OpaqueJSClassContextData : Noncopyable {
    OpaqueJSClassContextData(OpaqueJSClass*);
    ~OpaqueJSClassContextData();

    // It is necessary to keep OpaqueJSClass alive because of the following rare scenario:
    // 1. A class is created and used, so its context data is stored in JSGlobalData hash map.
    // 2. The class is released, and when all JS objects that use it are collected, OpaqueJSClass
    // is deleted (that's the part prevented by this RefPtr).
    // 3. Another class is created at the same address.
    // 4. When it is used, the old context data is found in JSGlobalData and used.
    RefPtr<OpaqueJSClass> m_class;

    OpaqueJSClassStaticValuesTable* staticValues;
    OpaqueJSClassStaticFunctionsTable* staticFunctions;
    JSC::WeakGCPtr<JSC::JSObject> cachedPrototype;
};

struct OpaqueJSClass : public ThreadSafeShared<OpaqueJSClass> {
    static PassRefPtr<OpaqueJSClass> create(const JSClassDefinition*);
    static PassRefPtr<OpaqueJSClass> createNoAutomaticPrototype(const JSClassDefinition*);
    ~OpaqueJSClass();
    
    JSC::UString className();
    OpaqueJSClassStaticValuesTable* staticValues(JSC::ExecState*);
    OpaqueJSClassStaticFunctionsTable* staticFunctions(JSC::ExecState*);
    JSC::JSObject* prototype(JSC::ExecState*);

    OpaqueJSClass* parentClass;
    OpaqueJSClass* prototypeClass;
    
    JSObjectInitializeCallback initialize;
    JSObjectFinalizeCallback finalize;
    JSObjectHasPropertyCallback hasProperty;
    JSObjectGetPropertyCallback getProperty;
    JSObjectSetPropertyCallback setProperty;
    JSObjectDeletePropertyCallback deleteProperty;
    JSObjectGetPropertyNamesCallback getPropertyNames;
    JSObjectCallAsFunctionCallback callAsFunction;
    JSObjectCallAsConstructorCallback callAsConstructor;
    JSObjectHasInstanceCallback hasInstance;
    JSObjectConvertToTypeCallback convertToType;

private:
    friend struct OpaqueJSClassContextData;

    OpaqueJSClass();
    OpaqueJSClass(const OpaqueJSClass&);
    OpaqueJSClass(const JSClassDefinition*, OpaqueJSClass* protoClass);

    OpaqueJSClassContextData& contextData(JSC::ExecState*);

    // UStrings in these data members should not be put into any IdentifierTable.
    JSC::UString m_className;
    OpaqueJSClassStaticValuesTable* m_staticValues;
    OpaqueJSClassStaticFunctionsTable* m_staticFunctions;
};

#endif // JSClassRef_h
