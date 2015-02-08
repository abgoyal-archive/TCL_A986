
#include "config.h"
#include "MessagePortChannel.h"

namespace WebCore {


PassOwnPtr<MessagePortChannel::EventData> MessagePortChannel::EventData::create(PassRefPtr<SerializedScriptValue> message, PassOwnPtr<MessagePortChannelArray> channels)
{
    return new EventData(message, channels);
}

MessagePortChannel::EventData::EventData(PassRefPtr<SerializedScriptValue> message, PassOwnPtr<MessagePortChannelArray> channels)
    : m_message(message->release())
    , m_channels(channels)
{
}

} // namespace WebCore
