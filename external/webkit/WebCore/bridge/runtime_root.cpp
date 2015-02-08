

#include "config.h"
#include "runtime_root.h"

#include "Bridge.h"
#include "runtime_object.h"
#include <runtime/JSGlobalObject.h>
#include <wtf/HashCountedSet.h>
#include <wtf/HashSet.h>
#include <wtf/StdLibExtras.h>

namespace JSC { namespace Bindings {

// This code attempts to solve two problems: (1) plug-ins leaking references to 
// JS and the DOM; (2) plug-ins holding stale references to JS and the DOM. Previous 
// comments in this file claimed that problem #1 was an issue in Java, in particular, 
// because Java, allegedly, didn't always call finalize when collecting an object.

typedef HashSet<RootObject*> RootObjectSet;

static RootObjectSet* rootObjectSet()
{
    DEFINE_STATIC_LOCAL(RootObjectSet, staticRootObjectSet, ());
    return &staticRootObjectSet;
}

// FIXME:  These two functions are a potential performance problem.  We could 
// fix them by adding a JSObject to RootObject dictionary.

RootObject* findProtectingRootObject(JSObject* jsObject)
{
    RootObjectSet::const_iterator end = rootObjectSet()->end();
    for (RootObjectSet::const_iterator it = rootObjectSet()->begin(); it != end; ++it) {
        if ((*it)->gcIsProtected(jsObject))
            return *it;
    }
    return 0;
}

RootObject* findRootObject(JSGlobalObject* globalObject)
{
    RootObjectSet::const_iterator end = rootObjectSet()->end();
    for (RootObjectSet::const_iterator it = rootObjectSet()->begin(); it != end; ++it) {
        if ((*it)->globalObject() == globalObject)
            return *it;
    }
    return 0;
}

PassRefPtr<RootObject> RootObject::create(const void* nativeHandle, JSGlobalObject* globalObject)
{
    return adoptRef(new RootObject(nativeHandle, globalObject));
}

RootObject::RootObject(const void* nativeHandle, JSGlobalObject* globalObject)
    : m_isValid(true)
    , m_nativeHandle(nativeHandle)
    , m_globalObject(globalObject)
{
    ASSERT(globalObject);
    rootObjectSet()->add(this);
}

RootObject::~RootObject()
{
    if (m_isValid)
        invalidate();
}

void RootObject::invalidate()
{
    if (!m_isValid)
        return;

    {
        HashSet<RuntimeObjectImp*>::iterator end = m_runtimeObjects.end();
        for (HashSet<RuntimeObjectImp*>::iterator it = m_runtimeObjects.begin(); it != end; ++it)
            (*it)->invalidate();
        
        m_runtimeObjects.clear();
    }
    
    m_isValid = false;

    m_nativeHandle = 0;
    m_globalObject = 0;

    ProtectCountSet::iterator end = m_protectCountSet.end();
    for (ProtectCountSet::iterator it = m_protectCountSet.begin(); it != end; ++it)
        JSC::gcUnprotect(it->first);
    m_protectCountSet.clear();

    rootObjectSet()->remove(this);
}

void RootObject::gcProtect(JSObject* jsObject)
{
    ASSERT(m_isValid);
    
    if (!m_protectCountSet.contains(jsObject))
        JSC::gcProtect(jsObject);
    m_protectCountSet.add(jsObject);
}

void RootObject::gcUnprotect(JSObject* jsObject)
{
    ASSERT(m_isValid);
    
    if (!jsObject)
        return;

    if (m_protectCountSet.count(jsObject) == 1)
        JSC::gcUnprotect(jsObject);
    m_protectCountSet.remove(jsObject);
}

bool RootObject::gcIsProtected(JSObject* jsObject)
{
    ASSERT(m_isValid);
    return m_protectCountSet.contains(jsObject);
}

const void* RootObject::nativeHandle() const 
{ 
    ASSERT(m_isValid);
    return m_nativeHandle; 
}

JSGlobalObject* RootObject::globalObject() const
{
    ASSERT(m_isValid);
    return m_globalObject;
}

void RootObject::addRuntimeObject(RuntimeObjectImp* object)
{
    ASSERT(m_isValid);
    ASSERT(!m_runtimeObjects.contains(object));
    
    m_runtimeObjects.add(object);
}        
    
void RootObject::removeRuntimeObject(RuntimeObjectImp* object)
{
    ASSERT(m_isValid);
    ASSERT(m_runtimeObjects.contains(object));
    
    m_runtimeObjects.remove(object);
}

} } // namespace JSC::Bindings
