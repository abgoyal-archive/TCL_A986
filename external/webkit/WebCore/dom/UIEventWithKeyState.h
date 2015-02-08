

#ifndef UIEventWithKeyState_h
#define UIEventWithKeyState_h

#include "UIEvent.h"

namespace WebCore {
    
    class UIEventWithKeyState : public UIEvent {
    public:
        bool ctrlKey() const { return m_ctrlKey; }
        bool shiftKey() const { return m_shiftKey; }
        bool altKey() const { return m_altKey; }
        bool metaKey() const { return m_metaKey; }

    protected:
        UIEventWithKeyState()
            : m_ctrlKey(false)
            , m_altKey(false)
            , m_shiftKey(false)
            , m_metaKey(false)
        {
        }
        
        UIEventWithKeyState(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView> view,
                            int detail, bool ctrlKey, bool altKey, bool shiftKey, bool metaKey)
            : UIEvent(type, canBubble, cancelable, view, detail)
            , m_ctrlKey(ctrlKey)
            , m_altKey(altKey)
            , m_shiftKey(shiftKey)
            , m_metaKey(metaKey)
        {
        }

        // Expose these so init functions can set them.
        bool m_ctrlKey : 1;
        bool m_altKey : 1;
        bool m_shiftKey : 1;
        bool m_metaKey : 1;
    };

    UIEventWithKeyState* findEventWithKeyState(Event*);

} // namespace WebCore

#endif // UIEventWithKeyState_h
