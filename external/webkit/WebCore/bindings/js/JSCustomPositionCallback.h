

#ifndef JSCustomPositionCallback_h
#define JSCustomPositionCallback_h

#include "JSCallbackData.h"
#include "PositionCallback.h"
#include <wtf/Forward.h>

namespace WebCore {

class Geoposition;
class JSDOMGlobalObject;

class JSCustomPositionCallback : public PositionCallback {
public:
    static PassRefPtr<JSCustomPositionCallback> create(JSC::JSObject* callback, JSDOMGlobalObject* globalObject)
    {
        return adoptRef(new JSCustomPositionCallback(callback, globalObject));
    }
    
private:
    JSCustomPositionCallback(JSC::JSObject* callback, JSDOMGlobalObject*);

    virtual void handleEvent(Geoposition*);
    
    JSCallbackData m_data;
};
    
} // namespace WebCore

#endif // JSCustomPositionCallback_h
