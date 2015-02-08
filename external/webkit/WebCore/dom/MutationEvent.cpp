

#include "config.h"
#include "MutationEvent.h"

namespace WebCore {

MutationEvent::MutationEvent()
    : m_attrChange(0)
{
}

MutationEvent::MutationEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<Node> relatedNode,
                             const String& prevValue, const String& newValue,
                             const String& attrName, unsigned short attrChange)
    : Event(type, canBubble, cancelable)
    , m_relatedNode(relatedNode)
    , m_prevValue(prevValue)
    , m_newValue(newValue)
    , m_attrName(attrName)
    , m_attrChange(attrChange)
{
}

void MutationEvent::initMutationEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<Node> relatedNode,
                                      const String& prevValue, const String& newValue,
                                      const String& attrName, unsigned short attrChange)
{
    if (dispatched())
        return;

    initEvent(type, canBubble, cancelable);

    m_relatedNode = relatedNode;
    m_prevValue = prevValue;
    m_newValue = newValue;
    m_attrName = attrName;
    m_attrChange = attrChange;
}

bool MutationEvent::isMutationEvent() const
{
    return true;
}

} // namespace WebCore
