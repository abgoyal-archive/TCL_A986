

#include "config.h"
#include "UIEvent.h"

#include "DOMWindow.h"

namespace WebCore {

UIEvent::UIEvent()
    : m_detail(0)
{
}

UIEvent::UIEvent(const AtomicString& eventType, bool canBubbleArg, bool cancelableArg, PassRefPtr<AbstractView> viewArg, int detailArg)
    : Event(eventType, canBubbleArg, cancelableArg)
    , m_view(viewArg)
    , m_detail(detailArg)
{
}

UIEvent::~UIEvent()
{
}

void UIEvent::initUIEvent(const AtomicString& typeArg, bool canBubbleArg, bool cancelableArg, PassRefPtr<AbstractView> viewArg, int detailArg)
{
    if (dispatched())
        return;

    initEvent(typeArg, canBubbleArg, cancelableArg);

    m_view = viewArg;
    m_detail = detailArg;
}

bool UIEvent::isUIEvent() const
{
    return true;
}

int UIEvent::keyCode() const
{
    return 0;
}

int UIEvent::charCode() const
{
    return 0;
}

int UIEvent::layerX() const
{
    return 0;
}

int UIEvent::layerY() const
{
    return 0;
}

int UIEvent::pageX() const
{
    return 0;
}

int UIEvent::pageY() const
{
    return 0;
}

int UIEvent::which() const
{
    return 0;
}

} // namespace WebCore
