

#ifndef AbstractWorker_h
#define AbstractWorker_h

#if ENABLE(WORKERS)

#include "ActiveDOMObject.h"
#include "AtomicStringHash.h"
#include "EventListener.h"
#include "EventNames.h"
#include "EventTarget.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

namespace WebCore {

    class KURL;
    class ScriptExecutionContext;

    class AbstractWorker : public RefCounted<AbstractWorker>, public ActiveDOMObject, public EventTarget {
    public:
        // EventTarget APIs
        virtual ScriptExecutionContext* scriptExecutionContext() const { return ActiveDOMObject::scriptExecutionContext(); }

        DEFINE_ATTRIBUTE_EVENT_LISTENER(error);

        using RefCounted<AbstractWorker>::ref;
        using RefCounted<AbstractWorker>::deref;

        AbstractWorker(ScriptExecutionContext*);
        virtual ~AbstractWorker();

    protected:
        // Helper function that converts a URL to an absolute URL and checks the result for validity.
        KURL resolveURL(const String& url, ExceptionCode& ec);

    private:
        virtual void refEventTarget() { ref(); }
        virtual void derefEventTarget() { deref(); }
        virtual EventTargetData* eventTargetData();
        virtual EventTargetData* ensureEventTargetData();
        
        EventTargetData m_eventTargetData;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // AbstractWorker_h
