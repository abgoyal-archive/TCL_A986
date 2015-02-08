

#ifndef ResourceError_h
#define ResourceError_h

#include "ResourceErrorBase.h"
#include <wtf/RetainPtr.h>

#ifdef __OBJC__
@class NSError;
#else
class NSError;
#endif

namespace WebCore {

    class ResourceError : public ResourceErrorBase {
    public:
        ResourceError()
            : m_dataIsUpToDate(true)
        {
        }

        ResourceError(const String& domain, int errorCode, const String& failingURL, const String& localizedDescription)
            : ResourceErrorBase(domain, errorCode, failingURL, localizedDescription)
            , m_dataIsUpToDate(true)
        {
        }

        ResourceError(NSError* error)
            : m_dataIsUpToDate(false)
            , m_platformError(error)
        {
            m_isNull = !error;
        }

        operator NSError*() const;

    private:
        friend class ResourceErrorBase;

        void platformLazyInit();
        static bool platformCompare(const ResourceError& a, const ResourceError& b);

        bool m_dataIsUpToDate;
        mutable RetainPtr<NSError> m_platformError;
};

} // namespace WebCore

#endif // ResourceError_h_
