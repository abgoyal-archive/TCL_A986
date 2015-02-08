

#ifndef SharedWorkerContext_h
#define SharedWorkerContext_h

#if ENABLE(SHARED_WORKERS)

#include "WorkerContext.h"

namespace WebCore {

    class MessageEvent;
    class SharedWorkerThread;

    class SharedWorkerContext : public WorkerContext {
    public:
        typedef WorkerContext Base;
        static PassRefPtr<SharedWorkerContext> create(const String& name, const KURL& url, const String& userAgent, SharedWorkerThread* thread)
        {
            return adoptRef(new SharedWorkerContext(name, url, userAgent, thread));
        }
        virtual ~SharedWorkerContext();

        virtual bool isSharedWorkerContext() const { return true; }

        // EventTarget
        virtual SharedWorkerContext* toSharedWorkerContext() { return this; }

        // Setters/Getters for attributes in SharedWorkerContext.idl
        DEFINE_ATTRIBUTE_EVENT_LISTENER(connect);
        String name() const { return m_name; }

        SharedWorkerThread* thread();
    private:
        SharedWorkerContext(const String& name, const KURL&, const String&, SharedWorkerThread*);
        String m_name;
    };

    PassRefPtr<MessageEvent> createConnectEvent(PassRefPtr<MessagePort>);

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)

#endif // SharedWorkerContext_h
