

#include "config.h"
#include "SocketStreamError.h"

namespace WebCore {

SocketStreamError SocketStreamErrorBase::copy() const
{
    SocketStreamError errorCopy;
    errorCopy.m_errorCode = m_errorCode;
    return errorCopy;
}

bool SocketStreamErrorBase::compare(const SocketStreamError& a, const SocketStreamError& b)
{
    return a.errorCode() == b.errorCode();
}

}  // namespace WebCore
