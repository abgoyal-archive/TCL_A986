

#ifndef V8CustomEventListener_h
#define V8CustomEventListener_h

#include "V8AbstractEventListener.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Event;
    class Frame;

    // V8EventListener is a wrapper of a JS object implements EventListener interface (has handleEvent(event) method), or a JS function
    // that can handle the event.
    class V8EventListener : public V8AbstractEventListener {
    public:
        static PassRefPtr<V8EventListener> create(v8::Local<v8::Object> listener, bool isAttribute, const WorldContextHandle& worldContext)
        {
            return adoptRef(new V8EventListener(listener, isAttribute, worldContext));
        }

    protected:
        V8EventListener(v8::Local<v8::Object> listener, bool isAttribute, const WorldContextHandle& worldContext);

        v8::Local<v8::Function> getListenerFunction(ScriptExecutionContext*);

    private:
      virtual v8::Local<v8::Value> callListenerFunction(ScriptExecutionContext*, v8::Handle<v8::Value> jsEvent, Event*);
     };

} // namespace WebCore

#endif // V8CustomEventListener_h
