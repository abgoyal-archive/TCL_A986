

#ifndef EventListener_h
#define EventListener_h

#include "PlatformString.h"
#include <wtf/RefCounted.h>

namespace JSC {
    class JSObject;
    class MarkStack;
}

namespace WebCore {

    class ScriptExecutionContext;
    class Event;

    class EventListener : public RefCounted<EventListener> {
    public:
        enum Type { JSEventListenerType, 
                    ImageEventListenerType, 
                    InspectorDOMAgentType,
                    InspectorDOMStorageResourceType,
                    ObjCEventListenerType, 
                    ConditionEventListenerType,
                    GeolocationEventListenerType };
                    
        virtual ~EventListener() { }
        virtual bool operator==(const EventListener&) = 0;
        virtual void handleEvent(ScriptExecutionContext*, Event*) = 0;
        // Return true to indicate that the error is handled.
        virtual bool reportError(ScriptExecutionContext*, const String& /*message*/, const String& /*url*/, int /*lineNumber*/) { return false; }
        virtual bool wasCreatedFromMarkup() const { return false; }

#if USE(JSC)
        virtual void markJSFunction(JSC::MarkStack&) { }
        virtual void invalidateJSFunction(JSC::JSObject*) { }
#endif

        bool isAttribute() const { return virtualisAttribute(); }
        Type type() const { return m_type; }

    protected:
        EventListener(Type type)
            : m_type(type)
        {
        }

    private:
        virtual bool virtualisAttribute() const { return false; }
        
        Type m_type;
    };

}

#endif
