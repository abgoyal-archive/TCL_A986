

#include "config.h"
#include "MouseEvent.h"

#include "EventNames.h"

namespace WebCore {

MouseEvent::MouseEvent()
    : m_button(0)
    , m_buttonDown(false)
{
}

MouseEvent::MouseEvent(const AtomicString& eventType, bool canBubble, bool cancelable, PassRefPtr<AbstractView> view,
                       int detail, int screenX, int screenY, int pageX, int pageY,
                       bool ctrlKey, bool altKey, bool shiftKey, bool metaKey,
                       unsigned short button, PassRefPtr<EventTarget> relatedTarget,
                       PassRefPtr<Clipboard> clipboard, bool isSimulated)
    : MouseRelatedEvent(eventType, canBubble, cancelable, view, detail, screenX, screenY,
                        pageX, pageY, ctrlKey, altKey, shiftKey, metaKey, isSimulated)
    , m_button(button == (unsigned short)-1 ? 0 : button)
    , m_buttonDown(button != (unsigned short)-1)
    , m_relatedTarget(relatedTarget)
    , m_clipboard(clipboard)
{
}

MouseEvent::~MouseEvent()
{
}

void MouseEvent::initMouseEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView> view,
                                int detail, int screenX, int screenY, int clientX, int clientY,
                                bool ctrlKey, bool altKey, bool shiftKey, bool metaKey,
                                unsigned short button, PassRefPtr<EventTarget> relatedTarget)
{
    if (dispatched())
        return;

    initUIEvent(type, canBubble, cancelable, view, detail);

    m_screenX = screenX;
    m_screenY = screenY;
    m_ctrlKey = ctrlKey;
    m_altKey = altKey;
    m_shiftKey = shiftKey;
    m_metaKey = metaKey;
    m_button = button == (unsigned short)-1 ? 0 : button;
    m_buttonDown = button != (unsigned short)-1;
    m_relatedTarget = relatedTarget;

    initCoordinates(clientX, clientY);

    // FIXME: m_isSimulated is not set to false here.
    // FIXME: m_clipboard is not set to 0 here.
}

bool MouseEvent::isMouseEvent() const
{
    return true;
}

bool MouseEvent::isDragEvent() const
{
    const AtomicString& t = type();
    return t == eventNames().dragenterEvent || t == eventNames().dragoverEvent || t == eventNames().dragleaveEvent || t == eventNames().dropEvent
               || t == eventNames().dragstartEvent|| t == eventNames().dragEvent || t == eventNames().dragendEvent;
}

int MouseEvent::which() const
{
    // For the DOM, the return values for left, middle and right mouse buttons are 0, 1, 2, respectively.
    // For the Netscape "which" property, the return values for left, middle and right mouse buttons are 1, 2, 3, respectively. 
    // So we must add 1.
    return m_button + 1;
}

Node* MouseEvent::toElement() const
{
    // MSIE extension - "the object toward which the user is moving the mouse pointer"
    if (type() == eventNames().mouseoutEvent) 
        return relatedTarget() ? relatedTarget()->toNode() : 0;
    
    return target() ? target()->toNode() : 0;
}

Node* MouseEvent::fromElement() const
{
    // MSIE extension - "object from which activation or the mouse pointer is exiting during the event" (huh?)
    if (type() != eventNames().mouseoutEvent)
        return relatedTarget() ? relatedTarget()->toNode() : 0;
    
    return target() ? target()->toNode() : 0;
}

} // namespace WebCore
