

#ifndef RUNTIME_ROOT_H_
#define RUNTIME_ROOT_H_

#if PLATFORM(MAC)
#include "jni_jsobject.h"
#endif
#include <runtime/Protect.h>

#include <wtf/Forward.h>
#include <wtf/HashSet.h>
#include <wtf/Noncopyable.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace JSC {

class Interpreter;
class JSGlobalObject;
class RuntimeObjectImp;

namespace Bindings {

class RootObject;

typedef HashCountedSet<JSObject*> ProtectCountSet;

extern RootObject* findProtectingRootObject(JSObject*);
extern RootObject* findRootObject(JSGlobalObject*);

class RootObject : public RefCounted<RootObject> {
    friend class JavaJSObject;

public:
    ~RootObject();
    
    static PassRefPtr<RootObject> create(const void* nativeHandle, JSGlobalObject*);

    bool isValid() { return m_isValid; }
    void invalidate();
    
    void gcProtect(JSObject*);
    void gcUnprotect(JSObject*);
    bool gcIsProtected(JSObject*);

    const void* nativeHandle() const;
    JSGlobalObject* globalObject() const;

    void addRuntimeObject(RuntimeObjectImp*);
    void removeRuntimeObject(RuntimeObjectImp*);
private:
    RootObject(const void* nativeHandle, JSGlobalObject*);
    
    bool m_isValid;
    
    const void* m_nativeHandle;
    ProtectedPtr<JSGlobalObject> m_globalObject;
    ProtectCountSet m_protectCountSet;

    HashSet<RuntimeObjectImp*> m_runtimeObjects;    
};

} // namespace Bindings

} // namespace JSC

#endif // RUNTIME_ROOT_H_
