

#ifndef XMLHttpRequestException_h
#define XMLHttpRequestException_h

#include "ExceptionBase.h"

namespace WebCore {

    class XMLHttpRequestException : public ExceptionBase {
    public:
        static PassRefPtr<XMLHttpRequestException> create(const ExceptionCodeDescription& description)
        {
            return adoptRef(new XMLHttpRequestException(description));
        }

        static const int XMLHttpRequestExceptionOffset = 500;
        static const int XMLHttpRequestExceptionMax = 699;

        enum XMLHttpRequestExceptionCode {
            NETWORK_ERR = XMLHttpRequestExceptionOffset + 101,
            ABORT_ERR
        };

    private:
        XMLHttpRequestException(const ExceptionCodeDescription& description)
            : ExceptionBase(description)
        {
        }
    };

} // namespace WebCore

#endif // XMLHttpRequestException_h
