

#ifndef JSCallbackData_h
#define JSCallbackData_h

#include "JSDOMBinding.h"
#include "JSDOMGlobalObject.h"
#include <runtime/JSObject.h>
#include <runtime/Protect.h>
#include <wtf/Threading.h>

namespace WebCore {

// We have to clean up this data on the main thread because unprotecting a
// JSObject on a non-main thread without synchronization would corrupt the heap
// (and synchronization would be slow).

class JSCallbackData {
public:
    static void deleteData(void*);

    JSCallbackData(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
        : m_callback(callback)
        , m_globalObject(globalObject)
    {
    }
    
    ~JSCallbackData()
    {
        ASSERT(isMainThread());
    }

    JSC::JSObject* callback() { return m_callback.get(); }
    JSDOMGlobalObject* globalObject() { return m_globalObject.get(); }
    
    JSC::JSValue invokeCallback(JSC::MarkedArgumentBuffer&, bool* raisedException = 0);

private:
    JSC::ProtectedPtr<JSC::JSObject> m_callback;
    JSC::ProtectedPtr<JSDOMGlobalObject> m_globalObject;
};

} // namespace WebCore

#endif // JSCallbackData_h
