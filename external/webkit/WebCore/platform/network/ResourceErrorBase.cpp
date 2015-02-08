

#include "config.h"
#include "ResourceError.h"

namespace WebCore {

ResourceError ResourceErrorBase::copy() const
{
    lazyInit();

    ResourceError errorCopy;
    errorCopy.m_domain = m_domain.crossThreadString();
    errorCopy.m_errorCode = m_errorCode;
    errorCopy.m_failingURL = m_failingURL.crossThreadString();
    errorCopy.m_localizedDescription = m_localizedDescription.crossThreadString();
    errorCopy.m_isNull = m_isNull;
    errorCopy.m_isCancellation = m_isCancellation;
    return errorCopy;
}

void ResourceErrorBase::lazyInit() const
{
    const_cast<ResourceError*>(static_cast<const ResourceError*>(this))->platformLazyInit();
}

bool ResourceErrorBase::compare(const ResourceError& a, const ResourceError& b)
{
    if (a.isNull() && b.isNull())
        return true;

    if (a.isNull() || b.isNull())
        return false;

    if (a.domain() != b.domain())
        return false;

    if (a.errorCode() != b.errorCode())
        return false;

    if (a.failingURL() != b.failingURL())
        return false;

    if (a.localizedDescription() != b.localizedDescription())
        return false;

    if (a.isCancellation() != b.isCancellation())
        return false;

    return platformCompare(a, b);
}

} // namespace WebCore
