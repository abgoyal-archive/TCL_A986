

#ifndef ClipboardEvent_h
#define ClipboardEvent_h

#include "Clipboard.h"
#include "Event.h"

namespace WebCore {

    class ClipboardEvent : public Event {
    public:
        static PassRefPtr<ClipboardEvent> create()
        {
            return adoptRef(new ClipboardEvent);
        }
        static PassRefPtr<ClipboardEvent> create(const AtomicString& type, bool canBubbleArg, bool cancelableArg, PassRefPtr<Clipboard> clipboardArg)
        {
            return adoptRef(new ClipboardEvent(type, canBubbleArg, cancelableArg, clipboardArg));
        }

        Clipboard* clipboard() const { return m_clipboard.get(); }

        virtual bool isClipboardEvent() const;

    private:
        ClipboardEvent();
        ClipboardEvent(const AtomicString& type, bool canBubbleArg, bool cancelableArg, PassRefPtr<Clipboard>);

        RefPtr<Clipboard> m_clipboard;
    };

} // namespace WebCore

#endif // ClipboardEvent_h
