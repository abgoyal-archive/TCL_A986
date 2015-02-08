

#include "config.h"
#include "MessageChannel.h"

#include "MessagePort.h"
#include "MessagePortChannel.h"

namespace WebCore {

MessageChannel::MessageChannel(ScriptExecutionContext* context)
    : m_port1(MessagePort::create(*context))
    , m_port2(MessagePort::create(*context))
{
    MessagePortChannel::createChannel(m_port1.get(), m_port2.get());
}

MessageChannel::~MessageChannel()
{
}

} // namespace WebCore
