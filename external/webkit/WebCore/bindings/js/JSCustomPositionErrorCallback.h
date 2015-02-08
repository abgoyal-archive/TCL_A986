

#ifndef JSCustomPositionErrorCallback_h
#define JSCustomPositionErrorCallback_h

#include "JSCallbackData.h"
#include "PositionErrorCallback.h"
#include <wtf/Forward.h>

namespace WebCore {
    
class PositionError;

class JSCustomPositionErrorCallback : public PositionErrorCallback {
public:
    static PassRefPtr<JSCustomPositionErrorCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomPositionErrorCallback(callback, globalObject));
    }
    
private:
    JSCustomPositionErrorCallback(JSC::JSObject* callback, JSDOMGlobalObject* globalObject);
    virtual void handleEvent(PositionError*);
    
    JSCallbackData m_data;
};
    
} // namespace WebCore

#endif // JSCustomPositionErrorCallback_h
