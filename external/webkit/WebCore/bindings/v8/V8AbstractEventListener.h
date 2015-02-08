

#ifndef V8AbstractEventListener_h
#define V8AbstractEventListener_h

#include "EventListener.h"
#include "WorldContextHandle.h"

#include <v8.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

namespace WebCore {

    class Event;
    class Frame;
    class V8Proxy;

    // There are two kinds of event listeners: HTML or non-HMTL. onload,
    // onfocus, etc (attributes) are always HTML event handler type; Event
    // listeners added by Window.addEventListener or
    // EventTargetNode::addEventListener are non-HTML type.
    //
    // Why does this matter?
    // WebKit does not allow duplicated HTML event handlers of the same type,
    // but ALLOWs duplicated non-HTML event handlers.
    class V8AbstractEventListener : public EventListener {
    public:
        virtual ~V8AbstractEventListener();

        static const V8AbstractEventListener* cast(const EventListener* listener)
        {
            return listener->type() == JSEventListenerType
                ? static_cast<const V8AbstractEventListener*>(listener)
                : 0;
        }

        static V8AbstractEventListener* cast(EventListener* listener)
        {
            return const_cast<V8AbstractEventListener*>(cast(const_cast<const EventListener*>(listener)));
        }

        // Implementation of EventListener interface.

        virtual bool operator==(const EventListener& other) { return this == &other; }

        virtual void handleEvent(ScriptExecutionContext*, Event*);

        virtual bool isLazy() const { return false; }

        // Returns the listener object, either a function or an object.
        v8::Local<v8::Object> getListenerObject(ScriptExecutionContext* context)
        {
            prepareListenerObject(context);
            return v8::Local<v8::Object>::New(m_listener);
        }

        v8::Local<v8::Object> getExistingListenerObject()
        {
            return v8::Local<v8::Object>::New(m_listener);
        }

        bool hasExistingListenerObject()
        {
            return !m_listener.IsEmpty();
        }

        // Dispose listener object and clear the handle.
        void disposeListenerObject();

    protected:
        V8AbstractEventListener(bool isAttribute, const WorldContextHandle& worldContext);

        virtual void prepareListenerObject(ScriptExecutionContext*) { }

        void setListenerObject(v8::Handle<v8::Object> listener);

        void invokeEventHandler(ScriptExecutionContext*, Event*, v8::Handle<v8::Value> jsEvent);

        // Get the receiver object to use for event listener call.
        v8::Local<v8::Object> getReceiverObject(Event*);

        const WorldContextHandle& worldContext() const { return m_worldContext; }

    private:
        // Implementation of EventListener function.
        virtual bool virtualisAttribute() const { return m_isAttribute; }

        virtual v8::Local<v8::Value> callListenerFunction(ScriptExecutionContext*, v8::Handle<v8::Value> jsevent, Event*) = 0;

        v8::Persistent<v8::Object> m_listener;

        // Indicates if the above handle is weak.
        bool m_isWeak;

        // Indicates if this is an HTML type listener.
        bool m_isAttribute;

        WorldContextHandle m_worldContext;
    };

} // namespace WebCore

#endif // V8AbstractEventListener_h
