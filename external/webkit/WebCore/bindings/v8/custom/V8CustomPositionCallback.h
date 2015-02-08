

#ifndef V8CustomPositionCallback_h
#define V8CustomPositionCallback_h

#include "PositionCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class Geoposition;

class V8CustomPositionCallback : public PositionCallback {
public:
    static PassRefPtr<V8CustomPositionCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomPositionCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomPositionCallback();

    virtual void handleEvent(Geoposition*);

private:
    V8CustomPositionCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

} // namespace WebCore

#endif // V8CustomPositionCallback_h
