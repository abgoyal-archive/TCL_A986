

#ifndef MouseEvent_h
#define MouseEvent_h

#include "Clipboard.h"
#include "MouseRelatedEvent.h"

namespace WebCore {

    // Introduced in DOM Level 2
    class MouseEvent : public MouseRelatedEvent {
    public:
        static PassRefPtr<MouseEvent> create()
        {
            return adoptRef(new MouseEvent);
        }
        static PassRefPtr<MouseEvent> create(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView> view,
            int detail, int screenX, int screenY, int pageX, int pageY,
            bool ctrlKey, bool altKey, bool shiftKey, bool metaKey, unsigned short button,
            PassRefPtr<EventTarget> relatedTarget, PassRefPtr<Clipboard> clipboard = 0, bool isSimulated = false)
        {
            return adoptRef(new MouseEvent(type, canBubble, cancelable, view, detail, screenX, screenY, pageX, pageY,
                ctrlKey, altKey, shiftKey, metaKey, button, relatedTarget, clipboard, isSimulated));
        }
        virtual ~MouseEvent();

        void initMouseEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView>,
                            int detail, int screenX, int screenY, int clientX, int clientY,
                            bool ctrlKey, bool altKey, bool shiftKey, bool metaKey,
                            unsigned short button, PassRefPtr<EventTarget> relatedTarget);

        // WinIE uses 1,4,2 for left/middle/right but not for click (just for mousedown/up, maybe others),
        // but we will match the standard DOM.
        unsigned short button() const { return m_button; }
        bool buttonDown() const { return m_buttonDown; }
        EventTarget* relatedTarget() const { return m_relatedTarget.get(); }

        Clipboard* clipboard() const { return m_clipboard.get(); }

        Node* toElement() const;
        Node* fromElement() const;

        Clipboard* dataTransfer() const { return isDragEvent() ? m_clipboard.get() : 0; }

        virtual bool isMouseEvent() const;
        virtual bool isDragEvent() const;
        virtual int which() const;

    private:
        MouseEvent();
        MouseEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView>,
                   int detail, int screenX, int screenY, int pageX, int pageY,
                   bool ctrlKey, bool altKey, bool shiftKey, bool metaKey, unsigned short button,
                   PassRefPtr<EventTarget> relatedTarget, PassRefPtr<Clipboard> clipboard, bool isSimulated);

        unsigned short m_button;
        bool m_buttonDown;
        RefPtr<EventTarget> m_relatedTarget;
        RefPtr<Clipboard> m_clipboard;
    };

} // namespace WebCore

#endif // MouseEvent_h
