

#ifndef DedicatedWorkerContext_h
#define DedicatedWorkerContext_h

#if ENABLE(WORKERS)

#include "MessagePort.h"
#include "WorkerContext.h"

namespace WebCore {

    class DedicatedWorkerThread;

    class DedicatedWorkerContext : public WorkerContext {
    public:
        typedef WorkerContext Base;
        static PassRefPtr<DedicatedWorkerContext> create(const KURL& url, const String& userAgent, DedicatedWorkerThread* thread)
        {
            return adoptRef(new DedicatedWorkerContext(url, userAgent, thread));
        }

        virtual bool isDedicatedWorkerContext() const { return true; }

        // Overridden to allow us to check our pending activity after executing imported script.
        virtual void importScripts(const Vector<String>& urls, const String& callerURL, int callerLine, ExceptionCode&);

        // EventTarget
        virtual DedicatedWorkerContext* toDedicatedWorkerContext() { return this; }
        void postMessage(PassRefPtr<SerializedScriptValue>, ExceptionCode&);
        void postMessage(PassRefPtr<SerializedScriptValue>, const MessagePortArray*, ExceptionCode&);
        // FIXME: remove this when we update the ObjC bindings (bug #28774).
        void postMessage(PassRefPtr<SerializedScriptValue>, MessagePort*, ExceptionCode&);

        DEFINE_ATTRIBUTE_EVENT_LISTENER(message);

        DedicatedWorkerThread* thread();
    private:
        DedicatedWorkerContext(const KURL&, const String&, DedicatedWorkerThread*);
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // DedicatedWorkerContext_h
