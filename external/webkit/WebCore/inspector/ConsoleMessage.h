

#ifndef ConsoleMessage_h
#define ConsoleMessage_h

#include "Console.h"
#include "ScriptObject.h"
#include "ScriptState.h"

#include <wtf/Vector.h>

namespace WebCore {
    class InspectorFrontend;
    class ScriptCallStack;
    class ScriptString;

    class ConsoleMessage : public Noncopyable {
    public:
        ConsoleMessage(MessageSource, MessageType, MessageLevel, const String& m, unsigned li, const String& u, unsigned g);        
        ConsoleMessage(MessageSource, MessageType, MessageLevel, ScriptCallStack*, unsigned g, bool storeTrace = false);

#if ENABLE(INSPECTOR)
        void addToConsole(InspectorFrontend* frontend);
        void updateRepeatCountInConsole(InspectorFrontend* frontend);
#endif
        void incrementCount() { ++m_repeatCount; };
        bool isEqual(ScriptState*, ConsoleMessage* msg) const;

        MessageSource source() const { return m_source; }
        const String& message() const { return m_message; }

    private:
        MessageSource m_source;
        MessageType m_type;
        MessageLevel m_level;
        String m_message;
#if ENABLE(INSPECTOR)
        Vector<ScriptValue> m_arguments;
        ScriptState* m_scriptState;
#endif
        Vector<ScriptString> m_frames;
        unsigned m_line;
        String m_url;
        unsigned m_groupLevel;
        unsigned m_repeatCount;
    };

} // namespace WebCore

#endif // ConsoleMessage_h
