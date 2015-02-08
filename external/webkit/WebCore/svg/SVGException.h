

#ifndef SVGException_h
#define SVGException_h

#include "ExceptionBase.h"

#if ENABLE(SVG)

namespace WebCore {

    class SVGException : public ExceptionBase {
    public:
        static PassRefPtr<SVGException> create(const ExceptionCodeDescription& description)
        {
            return adoptRef(new SVGException(description));
        }

        static const int SVGExceptionOffset = 300;
        static const int SVGExceptionMax = 399;

        enum SVGExceptionCode {
            SVG_WRONG_TYPE_ERR          = SVGExceptionOffset,
            SVG_INVALID_VALUE_ERR       = SVGExceptionOffset + 1,
            SVG_MATRIX_NOT_INVERTABLE   = SVGExceptionOffset + 2
        };

    private:
        SVGException(const ExceptionCodeDescription& description)
            : ExceptionBase(description)
        {
        }
    };

} // namespace WebCore

#endif // ENABLE(SVG)

#endif // SVGException_h
