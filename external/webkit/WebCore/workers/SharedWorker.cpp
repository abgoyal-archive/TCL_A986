

#include "config.h"

#if ENABLE(SHARED_WORKERS)

#include "SharedWorker.h"

#include "KURL.h"
#include "MessageChannel.h"
#include "MessagePort.h"
#include "SharedWorkerRepository.h"

namespace WebCore {

SharedWorker::SharedWorker(const String& url, const String& name, ScriptExecutionContext* context, ExceptionCode& ec)
    : AbstractWorker(context)
{
    RefPtr<MessageChannel> channel = MessageChannel::create(scriptExecutionContext());
    m_port = channel->port1();
    OwnPtr<MessagePortChannel> remotePort = channel->port2()->disentangle(ec);
    ASSERT(!ec);

    KURL scriptUrl = resolveURL(url, ec);
    if (ec)
        return;
    SharedWorkerRepository::connect(this, remotePort.release(), scriptUrl, name, ec);
}

SharedWorker::~SharedWorker()
{
}

} // namespace WebCore

#endif  // ENABLE(SHARED_WORKERS)
