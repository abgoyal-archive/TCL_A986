

#include "config.h"
#include "TextEvent.h"

#include "EventNames.h"

namespace WebCore {

TextEvent::TextEvent()
    : m_isLineBreak(false)
    , m_isBackTab(false)
{
}

TextEvent::TextEvent(PassRefPtr<AbstractView> view, const String& data)
    : UIEvent(eventNames().textInputEvent, true, true, view, 0)
    , m_data(data)
    , m_isLineBreak(false)
    , m_isBackTab(false)
{
}

TextEvent::~TextEvent()
{
}

void TextEvent::initTextEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView> view, const String& data)
{
    if (dispatched())
        return;

    initUIEvent(type, canBubble, cancelable, view, 0);

    m_data = data;
}

bool TextEvent::isTextEvent() const
{
    return true;
}

} // namespace WebCore
