

#ifndef DOMCoreException_h
#define DOMCoreException_h

#include "ExceptionBase.h"

namespace WebCore {

    class DOMCoreException : public ExceptionBase {
    public:
        static PassRefPtr<DOMCoreException> create(const ExceptionCodeDescription& description)
        {
            return adoptRef(new DOMCoreException(description));
        }

    private:
        DOMCoreException(const ExceptionCodeDescription& description)
            : ExceptionBase(description)
        {
        }
    };

} // namespace WebCore

#endif // DOMCoreException_h
