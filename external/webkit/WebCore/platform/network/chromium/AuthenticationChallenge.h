

#ifndef AuthenticationChallenge_h
#define AuthenticationChallenge_h

#include "AuthenticationChallengeBase.h"
#include "AuthenticationClient.h"
#include <wtf/RefPtr.h>

namespace WebCore {

    class AuthenticationChallenge : public AuthenticationChallengeBase {
    public:
        AuthenticationChallenge() {}
        AuthenticationChallenge(const ProtectionSpace&, const Credential& proposedCredential, unsigned previousFailureCount, const ResourceResponse&, const ResourceError&);

        AuthenticationClient* authenticationClient() const { return m_authenticationClient.get(); }

    private:
        friend class AuthenticationChallengeBase;
        static bool platformCompare(const AuthenticationChallenge&, const AuthenticationChallenge&);

        RefPtr<AuthenticationClient> m_authenticationClient;
    };

} // namespace WebCore

#endif
