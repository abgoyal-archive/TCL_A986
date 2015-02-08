

#ifndef ResourceError_h
#define ResourceError_h

#include "ResourceErrorBase.h"

namespace WebCore {

    class ResourceError : public ResourceErrorBase {
    public:
        ResourceError()
        {
        }

        ResourceError(const String& domain, int errorCode, const String& failingURL, const String& localizedDescription)
            : ResourceErrorBase(domain, errorCode, failingURL, localizedDescription)
        {
        }

    private:
        friend class ResourceErrorBase;
    };

} // namespace WebCore

#endif
