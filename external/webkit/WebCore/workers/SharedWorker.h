

#ifndef SharedWorker_h
#define SharedWorker_h

#include "AbstractWorker.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

#if ENABLE(SHARED_WORKERS)

namespace WebCore {

    class SharedWorker : public AbstractWorker {
    public:
        static PassRefPtr<SharedWorker> create(const String& url, const String& name, ScriptExecutionContext* context, ExceptionCode& ec)
        {
            return adoptRef(new SharedWorker(url, name, context, ec));
        }
        ~SharedWorker();
        MessagePort* port() const { return m_port.get(); }

        virtual SharedWorker* toSharedWorker() { return this; }

    private:
        SharedWorker(const String& url, const String& name, ScriptExecutionContext*, ExceptionCode&);

        RefPtr<MessagePort> m_port;
    };

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)

#endif // SharedWorker_h
