

#ifndef RangeException_h
#define RangeException_h

#include "ExceptionBase.h"

namespace WebCore {

    class RangeException : public ExceptionBase {
    public:
        static PassRefPtr<RangeException> create(const ExceptionCodeDescription& description)
        {
            return adoptRef(new RangeException(description));
        }

        static const int RangeExceptionOffset = 200;
        static const int RangeExceptionMax = 299;

        enum RangeExceptionCode {
            BAD_BOUNDARYPOINTS_ERR = RangeExceptionOffset + 1,
            INVALID_NODE_TYPE_ERR
        };

    private:
        RangeException(const ExceptionCodeDescription& description)
            : ExceptionBase(description)
        {
        }
    };

} // namespace WebCore

#endif // RangeException_h
