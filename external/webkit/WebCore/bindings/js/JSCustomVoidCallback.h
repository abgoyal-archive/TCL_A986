

#ifndef JSCustomVoidCallback_h
#define JSCustomVoidCallback_h

#include "JSDOMGlobalObject.h"
#include "VoidCallback.h"
#include <runtime/Protect.h>
#include <wtf/Forward.h>

namespace WebCore {

class JSCallbackData;

class JSCustomVoidCallback : public VoidCallback {
public: 
    static PassRefPtr<JSCustomVoidCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomVoidCallback(callback, globalObject));
    }
    
    virtual ~JSCustomVoidCallback();
    
    virtual void handleEvent();
    
private:
    JSCustomVoidCallback(JSC::JSObject* callback, JSDOMGlobalObject*);

    JSCallbackData* m_data;
};

} // namespace WebCore

#endif // JSCustomVoidCallback_h
