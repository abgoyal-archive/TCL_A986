

#ifndef TextEvent_h
#define TextEvent_h

#include "UIEvent.h"

namespace WebCore {

    class TextEvent : public UIEvent {
    public:
        static PassRefPtr<TextEvent> create()
        {
            return adoptRef(new TextEvent);
        }
        static PassRefPtr<TextEvent> create(PassRefPtr<AbstractView> view, const String& data)
        {
            return adoptRef(new TextEvent(view, data));
        }
        virtual ~TextEvent();
    
        void initTextEvent(const AtomicString& type, bool canBubble, bool cancelable, PassRefPtr<AbstractView>, const String& data);
    
        String data() const { return m_data; }

        virtual bool isTextEvent() const;

        // If true, any newline characters in the text are line breaks only, not paragraph separators.
        bool isLineBreak() const { return m_isLineBreak; }
        void setIsLineBreak(bool isLineBreak) { m_isLineBreak = isLineBreak; }

        // If true, any tab characters in the text are backtabs.
        bool isBackTab() const { return m_isBackTab; }
        void setIsBackTab(bool isBackTab) { m_isBackTab = isBackTab; }

    private:
        TextEvent();
        TextEvent(PassRefPtr<AbstractView>, const String& data);

        String m_data;
        bool m_isLineBreak;
        bool m_isBackTab;
    };

} // namespace WebCore

#endif // TextEvent_h
