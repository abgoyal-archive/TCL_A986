

#include "config.h"

#if ENABLE(SHARED_WORKERS)

#include "SharedWorkerContext.h"

#include "DOMWindow.h"
#include "EventNames.h"
#include "MessageEvent.h"
#include "NotImplemented.h"
#include "SharedWorkerThread.h"

namespace WebCore {

PassRefPtr<MessageEvent> createConnectEvent(PassRefPtr<MessagePort> port)
{
    RefPtr<MessageEvent> event = MessageEvent::create(new MessagePortArray(1, port));
    event->initEvent(eventNames().connectEvent, false, false);
    return event;
}

SharedWorkerContext::SharedWorkerContext(const String& name, const KURL& url, const String& userAgent, SharedWorkerThread* thread)
    : WorkerContext(url, userAgent, thread)
    , m_name(name)
{
}

SharedWorkerContext::~SharedWorkerContext()
{
}

SharedWorkerThread* SharedWorkerContext::thread()
{
    return static_cast<SharedWorkerThread*>(Base::thread());
}

} // namespace WebCore

#endif // ENABLE(SHARED_WORKERS)
