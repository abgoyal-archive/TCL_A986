
#ifndef AuthenticationChallengeBase_h
#define AuthenticationChallengeBase_h

#include "Credential.h"
#include "ProtectionSpace.h"
#include "ResourceResponse.h"
#include "ResourceError.h"

namespace WebCore {

class AuthenticationChallenge;

class AuthenticationChallengeBase {
public:
    AuthenticationChallengeBase();
    AuthenticationChallengeBase(const ProtectionSpace& protectionSpace, const Credential& proposedCredential, unsigned previousFailureCount, const ResourceResponse& response, const ResourceError& error);

    unsigned previousFailureCount() const;
    const Credential& proposedCredential() const;
    const ProtectionSpace& protectionSpace() const;
    const ResourceResponse& failureResponse() const;
    const ResourceError& error() const;
    
    bool isNull() const;
    void nullify();

    static bool compare(const AuthenticationChallenge& a, const AuthenticationChallenge& b);

protected:
    // The AuthenticationChallenge subclass may "shadow" this method to compare platform specific fields
    static bool platformCompare(const AuthenticationChallengeBase&, const AuthenticationChallengeBase&) { return true; }

    bool m_isNull;
    ProtectionSpace m_protectionSpace;
    Credential m_proposedCredential;
    unsigned m_previousFailureCount;
    ResourceResponse m_failureResponse;
    ResourceError m_error;
};

inline bool operator==(const AuthenticationChallenge& a, const AuthenticationChallenge& b) { return AuthenticationChallengeBase::compare(a, b); }
inline bool operator!=(const AuthenticationChallenge& a, const AuthenticationChallenge& b) { return !(a == b); }

}

#endif
