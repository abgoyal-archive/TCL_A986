

#ifndef XMLHttpRequestUpload_h
#define XMLHttpRequestUpload_h

#include "AtomicStringHash.h"
#include "EventListener.h"
#include "EventNames.h"
#include "EventTarget.h"
#include <wtf/HashMap.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

    class AtomicStringImpl;
    class ScriptExecutionContext;
    class XMLHttpRequest;

    class XMLHttpRequestUpload : public RefCounted<XMLHttpRequestUpload>, public EventTarget {
    public:
        static PassRefPtr<XMLHttpRequestUpload> create(XMLHttpRequest* xmlHttpRequest)
        {
            return adoptRef(new XMLHttpRequestUpload(xmlHttpRequest));
        }

        virtual XMLHttpRequestUpload* toXMLHttpRequestUpload() { return this; }

        XMLHttpRequest* associatedXMLHttpRequest() const { return m_xmlHttpRequest; }
        void disconnectXMLHttpRequest() { m_xmlHttpRequest = 0; }

        ScriptExecutionContext* scriptExecutionContext() const;

        DEFINE_ATTRIBUTE_EVENT_LISTENER(abort);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(error);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(load);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(loadstart);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(progress);

        using RefCounted<XMLHttpRequestUpload>::ref;
        using RefCounted<XMLHttpRequestUpload>::deref;

    private:
        XMLHttpRequestUpload(XMLHttpRequest*);

        virtual void refEventTarget() { ref(); }
        virtual void derefEventTarget() { deref(); }
        virtual EventTargetData* eventTargetData();
        virtual EventTargetData* ensureEventTargetData();

        XMLHttpRequest* m_xmlHttpRequest;
        EventTargetData m_eventTargetData;
    };
    
} // namespace WebCore

#endif // XMLHttpRequestUpload_h
