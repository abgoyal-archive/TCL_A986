

#ifndef V8CustomVoidCallback_h
#define V8CustomVoidCallback_h

#include "VoidCallback.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class Frame;

class V8CustomVoidCallback : public VoidCallback {
public:
    static PassRefPtr<V8CustomVoidCallback> create(v8::Local<v8::Value> value, Frame* frame)
    {
        ASSERT(value->IsObject());
        return adoptRef(new V8CustomVoidCallback(value->ToObject(), frame));
    }
    virtual ~V8CustomVoidCallback();

    virtual void handleEvent();

private:
    V8CustomVoidCallback(v8::Local<v8::Object>, Frame*);

    v8::Persistent<v8::Object> m_callback;
    RefPtr<Frame> m_frame;
};

// Returns false if callback failed (null, wrong type, or threw exception).
bool invokeCallback(v8::Persistent<v8::Object> callback, int argc, v8::Handle<v8::Value> argv[], bool& callbackReturnValue);

} // namespace WebCore

#endif // V8CustomVoidCallback_h
