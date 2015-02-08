

#include "config.h"
#include "AuthenticationChallenge.h"

namespace WebCore {

bool AuthenticationChallenge::platformCompare(const AuthenticationChallenge&, const AuthenticationChallenge&)
{
    return true;
}

} // namespace WebCore
