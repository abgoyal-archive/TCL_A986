

#ifndef ErrorEvent_h
#define ErrorEvent_h

#if ENABLE(WORKERS)

#include "Event.h"
#include "PlatformString.h"

namespace WebCore {

    class ErrorEvent : public Event {
    public:
        static PassRefPtr<ErrorEvent> create()
        {
            return adoptRef(new ErrorEvent);
        }
        static PassRefPtr<ErrorEvent> create(const String& message, const String& fileName, unsigned lineNumber)
        {
            return adoptRef(new ErrorEvent(message, fileName, lineNumber));
        }
        virtual ~ErrorEvent();

        void initErrorEvent(const AtomicString& type, bool canBubble, bool cancelable, const String& message, const String& fileName, unsigned lineNumber);

        const String& message() const { return m_message; }
        const String& filename() const { return m_fileName; }
        unsigned lineno() const { return m_lineNumber; }

        virtual bool isErrorEvent() const;

    private:    
        ErrorEvent();
        ErrorEvent(const String& message, const String& fileName, unsigned lineNumber);

        String m_message;
        String m_fileName;
        unsigned m_lineNumber;
    };

} // namespace WebCore

#endif // ENABLE(WORKERS)

#endif // ErrorEvent_h
