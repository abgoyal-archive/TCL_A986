

#ifndef V8WorkerContextEventListener_h
#define V8WorkerContextEventListener_h

#if ENABLE(WORKERS)

#include "V8CustomEventListener.h"
#include <v8.h>
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class Event;
    class WorkerContextExecutionProxy;

    class V8WorkerContextEventListener : public V8EventListener {
    public:
        static PassRefPtr<V8WorkerContextEventListener> create(v8::Local<v8::Object> listener, bool isInline, const WorldContextHandle& worldContext)
        {
            return adoptRef(new V8WorkerContextEventListener(listener, isInline, worldContext));
        }

        virtual void handleEvent(ScriptExecutionContext*, Event*);
        virtual bool reportError(ScriptExecutionContext*, const String& message, const String& url, int lineNumber);

    private:
        V8WorkerContextEventListener(v8::Local<v8::Object> listener, bool isInline, const WorldContextHandle& worldContext);

        virtual v8::Local<v8::Value> callListenerFunction(ScriptExecutionContext*, v8::Handle<v8::Value> jsEvent, Event*);
        v8::Local<v8::Object> getReceiverObject(ScriptExecutionContext*, Event*);
    };

} // namespace WebCore

#endif // WORKERS

#endif // V8WorkerContextEventListener_h
