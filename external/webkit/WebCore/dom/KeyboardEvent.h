

#ifndef KeyboardEvent_h
#define KeyboardEvent_h

#include "UIEventWithKeyState.h"
#include <wtf/Vector.h>

namespace WebCore {

    class PlatformKeyboardEvent;

#if PLATFORM(MAC)
    struct KeypressCommand {
        KeypressCommand(const String& commandName) : commandName(commandName) {}
        KeypressCommand(const String& commandName, const String& text) : commandName(commandName), text(text) { ASSERT(commandName == "insertText:"); }

        String commandName;
        String text;
    };
#endif
    
    // Introduced in DOM Level 3
    class KeyboardEvent : public UIEventWithKeyState {
    public:
        enum KeyLocationCode {
            DOM_KEY_LOCATION_STANDARD      = 0x00,
            DOM_KEY_LOCATION_LEFT          = 0x01,
            DOM_KEY_LOCATION_RIGHT         = 0x02,
            DOM_KEY_LOCATION_NUMPAD        = 0x03
        };
        
        static PassRefPtr<KeyboardEvent> create()
        {
            return adoptRef(new KeyboardEvent);
        }
        static PassRefPtr<KeyboardEvent> create(const PlatformKeyboardEvent& platformEvent, AbstractView* view)
        {
            return adoptRef(new KeyboardEvent(platformEvent, view));
        }
        static PassRefPtr<KeyboardEvent> create(const AtomicString& type, bool canBubble, bool cancelable, AbstractView* view,
            const String& keyIdentifier, unsigned keyLocation,
            bool ctrlKey, bool altKey, bool shiftKey, bool metaKey, bool altGraphKey)
        {
            return adoptRef(new KeyboardEvent(type, canBubble, cancelable, view, keyIdentifier, keyLocation,
                ctrlKey, altKey, shiftKey, metaKey, altGraphKey));
        }
        virtual ~KeyboardEvent();
    
        void initKeyboardEvent(const AtomicString& type, bool canBubble, bool cancelable, AbstractView*,
                               const String& keyIdentifier, unsigned keyLocation,
                               bool ctrlKey, bool altKey, bool shiftKey, bool metaKey, bool altGraphKey = false);
    
        String keyIdentifier() const { return m_keyIdentifier; }
        unsigned keyLocation() const { return m_keyLocation; }

        bool getModifierState(const String& keyIdentifier) const;

        bool altGraphKey() const { return m_altGraphKey; }
    
        const PlatformKeyboardEvent* keyEvent() const { return m_keyEvent.get(); }

        int keyCode() const; // key code for keydown and keyup, character for keypress
        int charCode() const; // character code for keypress, 0 for keydown and keyup
    
        virtual bool isKeyboardEvent() const;
        virtual int which() const;

#if PLATFORM(MAC)
        // We only have this need to store keypress command info on the Mac.
        Vector<KeypressCommand>& keypressCommands() { return m_keypressCommands; }
#endif

    private:
        KeyboardEvent();
        KeyboardEvent(const PlatformKeyboardEvent&, AbstractView*);
        KeyboardEvent(const AtomicString& type, bool canBubble, bool cancelable, AbstractView*,
                      const String& keyIdentifier, unsigned keyLocation,
                      bool ctrlKey, bool altKey, bool shiftKey, bool metaKey, bool altGraphKey);

        OwnPtr<PlatformKeyboardEvent> m_keyEvent;
        String m_keyIdentifier;
        unsigned m_keyLocation;
        bool m_altGraphKey : 1;

#if PLATFORM(MAC)        
        Vector<KeypressCommand> m_keypressCommands;
#endif
    };

    KeyboardEvent* findKeyboardEvent(Event*);

} // namespace WebCore

#endif // KeyboardEvent_h
