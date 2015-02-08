

#ifndef UIEvent_h
#define UIEvent_h

#include "DOMWindow.h"
#include "Event.h"

namespace WebCore {

    typedef DOMWindow AbstractView;

    class UIEvent : public Event {
    public:
        static PassRefPtr<UIEvent> create()
        {
            return adoptRef(new UIEvent);
        }
        static PassRefPtr<UIEvent> create(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView> view, int detail)
        {
            return adoptRef(new UIEvent(type, canBubble, cancelable, view, detail));
        }
        virtual ~UIEvent();

        void initUIEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView>, int detail);

        AbstractView* view() const { return m_view.get(); }
        int detail() const { return m_detail; }
        
        virtual bool isUIEvent() const;

        virtual int keyCode() const;
        virtual int charCode() const;

        virtual int layerX() const;
        virtual int layerY() const;

        virtual int pageX() const;
        virtual int pageY() const;

        virtual int which() const;

    protected:
        UIEvent();
        UIEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView>, int detail);

    private:
        RefPtr<AbstractView> m_view;
        int m_detail;
    };

} // namespace WebCore

#endif // UIEvent_h
