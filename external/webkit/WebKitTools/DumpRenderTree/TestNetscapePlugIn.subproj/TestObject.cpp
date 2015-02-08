

#include "TestObject.h"
#include "PluginObject.h"

#include <string.h>
#include <stdlib.h>

static bool testEnumerate(NPObject *npobj, NPIdentifier **value, uint32_t *count);
static bool testHasMethod(NPObject*, NPIdentifier name);
static bool testInvoke(NPObject*, NPIdentifier name, const NPVariant* args, uint32_t argCount, NPVariant* result);
static bool testHasProperty(NPObject*, NPIdentifier name);
static bool testGetProperty(NPObject*, NPIdentifier name, NPVariant*);
static NPObject *testAllocate(NPP npp, NPClass *theClass);
static void testDeallocate(NPObject *obj);
static bool testConstruct(NPObject* obj, const NPVariant* args, uint32_t argCount, NPVariant* result);

static NPClass testClass = { 
    NP_CLASS_STRUCT_VERSION,
    testAllocate, 
    testDeallocate, 
    0,
    testHasMethod,
    testInvoke,
    0,
    testHasProperty,
    testGetProperty,
    0,
    0,
    testEnumerate,
    testConstruct
};

NPClass *getTestClass(void)
{
    return &testClass;
}

static bool identifiersInitialized = false;

#define ID_OBJECT_POINTER 2

#define NUM_ENUMERATABLE_TEST_IDENTIFIERS 2
#define NUM_TEST_IDENTIFIERS 3

static NPIdentifier testIdentifiers[NUM_TEST_IDENTIFIERS];
static const NPUTF8 *testIdentifierNames[NUM_TEST_IDENTIFIERS] = {
    "foo",
    "bar",
    "objectPointer",
};

#define ID_THROW_EXCEPTION_METHOD   0
#define NUM_METHOD_IDENTIFIERS      1

static NPIdentifier testMethodIdentifiers[NUM_METHOD_IDENTIFIERS];
static const NPUTF8 *testMethodIdentifierNames[NUM_METHOD_IDENTIFIERS] = {
    "throwException",
};

static void initializeIdentifiers(void)
{
    browser->getstringidentifiers(testIdentifierNames, NUM_TEST_IDENTIFIERS, testIdentifiers);
    browser->getstringidentifiers(testMethodIdentifierNames, NUM_METHOD_IDENTIFIERS, testMethodIdentifiers);
}

static NPObject *testAllocate(NPP /*npp*/, NPClass* /*theClass*/)
{
    NPObject *newInstance = static_cast<NPObject*>(malloc(sizeof(NPObject)));
    
    if (!identifiersInitialized) {
        identifiersInitialized = true;
        initializeIdentifiers();
    }
    
    return newInstance;
}

static void testDeallocate(NPObject *obj) 
{
    free(obj);
}

static bool testHasMethod(NPObject*, NPIdentifier name)
{
    for (unsigned i = 0; i < NUM_METHOD_IDENTIFIERS; i++) {
        if (testMethodIdentifiers[i] == name)
            return true;
    }
    return false;
}

static bool testInvoke(NPObject* header, NPIdentifier name, const NPVariant* /*args*/, uint32_t /*argCount*/, NPVariant* /*result*/)
{
    if (name == testMethodIdentifiers[ID_THROW_EXCEPTION_METHOD]) {
        browser->setexception(header, "test object throwException SUCCESS");
        return true;
     }
     return false;
}

static bool testHasProperty(NPObject*, NPIdentifier name)
{
    for (unsigned i = 0; i < NUM_TEST_IDENTIFIERS; i++) {
        if (testIdentifiers[i] == name)
            return true;
    }
    
    return false;
}

static bool testGetProperty(NPObject* npobj, NPIdentifier name, NPVariant* result)
{
    if (name == testIdentifiers[ID_OBJECT_POINTER]) {
        int32_t objectPointer = static_cast<int32_t>(reinterpret_cast<long long>(npobj));

        INT32_TO_NPVARIANT(objectPointer, *result);
        return true;
    }
    
    return false;
}


static bool testEnumerate(NPObject* /*npobj*/, NPIdentifier **value, uint32_t *count)
{
    *count = NUM_ENUMERATABLE_TEST_IDENTIFIERS;
    
    *value = (NPIdentifier*)browser->memalloc(NUM_ENUMERATABLE_TEST_IDENTIFIERS * sizeof(NPIdentifier));
    memcpy(*value, testIdentifiers, sizeof(NPIdentifier) * NUM_ENUMERATABLE_TEST_IDENTIFIERS);
    
    return true;
}

static bool testConstruct(NPObject* npobj, const NPVariant* /*args*/, uint32_t /*argCount*/, NPVariant* result)
{
    browser->retainobject(npobj);
    
    // Just return the same object.
    OBJECT_TO_NPVARIANT(npobj, *result);
    return true;
}


