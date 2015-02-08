

#ifndef EventException_h
#define EventException_h

#include "ExceptionBase.h"

namespace WebCore {

    class EventException : public ExceptionBase {
    public:
        static PassRefPtr<EventException> create(const ExceptionCodeDescription& description)
        {
            return adoptRef(new EventException(description));
        }

        static const int EventExceptionOffset = 100;
        static const int EventExceptionMax = 199;

        enum EventExceptionCode {
            UNSPECIFIED_EVENT_TYPE_ERR = EventExceptionOffset
        };

    private:
        EventException(const ExceptionCodeDescription& description)
            : ExceptionBase(description)
        {
        }
    };

} // namespace WebCore

#endif // EventException_h
