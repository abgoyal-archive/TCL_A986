
#ifndef AuthenticationChallenge_h
#define AuthenticationChallenge_h

#include "AuthenticationChallengeBase.h"
#include "AuthenticationClient.h"
#include <wtf/RefPtr.h>

typedef struct _CFURLAuthChallenge* CFURLAuthChallengeRef;

namespace WebCore {

class AuthenticationChallenge : public AuthenticationChallengeBase {
public:
    AuthenticationChallenge() {}
    AuthenticationChallenge(const ProtectionSpace& protectionSpace, const Credential& proposedCredential, unsigned previousFailureCount, const ResourceResponse& response, const ResourceError& error);
    AuthenticationChallenge(CFURLAuthChallengeRef, AuthenticationClient*);

    AuthenticationClient* authenticationClient() const { return m_authenticationClient.get(); }
    void setAuthenticationClient(AuthenticationClient* client) { m_authenticationClient = client; }

    CFURLAuthChallengeRef cfURLAuthChallengeRef() const { return m_cfChallenge.get(); }

private:
    friend class AuthenticationChallengeBase;
    static bool platformCompare(const AuthenticationChallenge& a, const AuthenticationChallenge& b);

    RefPtr<AuthenticationClient> m_authenticationClient;
    RetainPtr<CFURLAuthChallengeRef> m_cfChallenge;
};

}

#endif
