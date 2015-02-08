

#ifndef V8ConsoleMessage_h
#define V8ConsoleMessage_h

#include "PlatformString.h"
#include <v8.h>
#include <wtf/Vector.h>

namespace WebCore {
    
    class Page;

    // V8ConsoleMessage encapsulates everything needed to
    // log messages originating from JavaScript to the console.
    class V8ConsoleMessage {
    public:
        V8ConsoleMessage(const String& string, const String& sourceID, unsigned lineNumber);

        // Add a message to the console. May end up calling JavaScript code
        // indirectly through the inspector so only call this function when
        // it is safe to do allocations.
        void dispatchNow(Page*);

        // Add a message to the console but delay the reporting until it
        // is safe to do so: Either when we leave JavaScript execution or
        // when adding other console messages. The primary purpose of this
        // method is to avoid calling into V8 to handle console messages
        // when the VM is in a state that does not support GCs or allocations.
        // Delayed messages are always reported in the page corresponding
        // to the active context.
        void dispatchLater();

        // Process any delayed messages. May end up calling JavaScript code
        // indirectly through the inspector so only call this function when
        // it is safe to do allocations.
        static void processDelayed();

        // Convenience class for ensuring that delayed messages in the
        // ConsoleMessageManager are processed quickly.
        class Scope {
        public:
            Scope() { V8ConsoleMessage::processDelayed(); }
            ~Scope() { V8ConsoleMessage::processDelayed(); }
        };

        // Callback from V8.
        static void handler(v8::Handle<v8::Message>, v8::Handle<v8::Value> data);

    private:
        const String m_string;
        const String m_sourceID;
        const unsigned m_lineNumber;

        // All delayed messages are stored in this vector. If the vector
        // is 0, there are no delayed messages.
        static Vector<V8ConsoleMessage>* m_delayedMessages;
    };

} // namespace WebCore

#endif // V8ConsoleMessage_h
