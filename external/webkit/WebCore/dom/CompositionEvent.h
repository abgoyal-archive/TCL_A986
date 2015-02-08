

#ifndef CompositionEvent_h
#define CompositionEvent_h

#include "UIEvent.h"

namespace WebCore {

    class CompositionEvent : public UIEvent {
    public:
        static PassRefPtr<CompositionEvent> create()
        {
            return adoptRef(new CompositionEvent);
        }
        static PassRefPtr<CompositionEvent> create(const AtomicString& type, PassRefPtr<AbstractView> view, const String& data)
        {
          return adoptRef(new CompositionEvent(type, view, data));
        }
        virtual ~CompositionEvent();
    
        void initCompositionEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView>, const String& data);
    
        String data() const { return m_data; }

        virtual bool isCompositionEvent() const;

    private:
        CompositionEvent();
        CompositionEvent(const AtomicString& type, PassRefPtr<AbstractView> view, const String& data);

        String m_data;
    };

} // namespace WebCore

#endif // CompositionEvent_h
