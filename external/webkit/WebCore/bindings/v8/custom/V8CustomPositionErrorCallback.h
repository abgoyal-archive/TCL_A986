

#ifndef V8CustomPositionErrorCallback_h
#define V8CustomPositionErrorCallback_h

#include "PositionErrorCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;
class PositionError;

class V8CustomPositionErrorCallback : public PositionErrorCallback {
public:
    static PassRefPtr<V8CustomPositionErrorCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomPositionErrorCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomPositionErrorCallback();

    virtual void handleEvent(PositionError*);

private:
    V8CustomPositionErrorCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

} // namespace WebCore

#endif // V8CustomPositionErrorCallback_h
