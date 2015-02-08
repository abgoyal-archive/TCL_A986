

#ifndef XPathException_h
#define XPathException_h

#include "ExceptionBase.h"

#if ENABLE(XPATH)

namespace WebCore {

    class XPathException : public ExceptionBase {
    public:
        static PassRefPtr<XPathException> create(const ExceptionCodeDescription& description)
        {
            return adoptRef(new XPathException(description));
        }

        static const int XPathExceptionOffset = 400;
        static const int XPathExceptionMax = 499;

        enum XPathExceptionCode {
            INVALID_EXPRESSION_ERR = XPathExceptionOffset + 51,
            TYPE_ERR
        };

    private:
        XPathException(const ExceptionCodeDescription& description)
            : ExceptionBase(description)
        {
        }
    };

} // namespace WebCore

#endif // ENABLE(XPATH)

#endif // XPathException_h
