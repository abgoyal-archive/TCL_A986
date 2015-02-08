
#ifndef AuthenticationChallenge_h
#define AuthenticationChallenge_h

#include "AuthenticationChallengeBase.h"
#include "AuthenticationClient.h"
#include <wtf/RefPtr.h>

namespace WebCore {

class AuthenticationChallenge : public AuthenticationChallengeBase {
public:
    AuthenticationChallenge()
    {
    }

    AuthenticationChallenge(const ProtectionSpace& protectionSpace, const Credential& proposedCredential, unsigned previousFailureCount, const ResourceResponse& response, const ResourceError& error)
        : AuthenticationChallengeBase(protectionSpace, proposedCredential, previousFailureCount, response, error)
    {
    }

    AuthenticationClient* authenticationClient() const { return m_authenticationClient.get(); }

    RefPtr<AuthenticationClient> m_authenticationClient;
};

}

#endif
