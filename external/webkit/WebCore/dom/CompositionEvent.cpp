

#include "config.h"
#include "CompositionEvent.h"

#include "EventNames.h"

namespace WebCore {

CompositionEvent::CompositionEvent()
{
}

CompositionEvent::CompositionEvent(const AtomicString& type, PassRefPtr<AbstractView> view, const String& data)
    : UIEvent(type, true, true, view, 0)
    , m_data(data)
{
}

CompositionEvent::~CompositionEvent()
{
}

void CompositionEvent::initCompositionEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView> view, const String& data)
{
    if (dispatched())
        return;

    initUIEvent(type, canBubble, cancelable, view, 0);

    m_data = data;
}

bool CompositionEvent::isCompositionEvent() const
{
    return true;
}

} // namespace WebCore
