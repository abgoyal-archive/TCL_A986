

#include "config.h"
#include "ClipboardEvent.h"

namespace WebCore {

ClipboardEvent::ClipboardEvent()
{
}

ClipboardEvent::ClipboardEvent(const AtomicString& eventType, bool canBubble, bool cancelable, PassRefPtr<Clipboard> clipboard)
    : Event(eventType, canBubble, cancelable), m_clipboard(clipboard)
{
}

bool ClipboardEvent::isClipboardEvent() const
{
    return true;
}

} // namespace WebCore
