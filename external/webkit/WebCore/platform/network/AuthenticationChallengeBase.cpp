
#include "config.h"
#include "AuthenticationChallenge.h"

#include "ResourceHandle.h"

namespace WebCore {

AuthenticationChallengeBase::AuthenticationChallengeBase()
    : m_isNull(true)
    , m_previousFailureCount(0)
{
}

AuthenticationChallengeBase::AuthenticationChallengeBase(const ProtectionSpace& protectionSpace,
                                                         const Credential& proposedCredential,
                                                         unsigned previousFailureCount,
                                                         const ResourceResponse& response,
                                                         const ResourceError& error)
    : m_isNull(false)
    , m_protectionSpace(protectionSpace)
    , m_proposedCredential(proposedCredential)
    , m_previousFailureCount(previousFailureCount)
    , m_failureResponse(response)
    , m_error(error)
{
}

unsigned AuthenticationChallengeBase::previousFailureCount() const 
{ 
    return m_previousFailureCount; 
}

const Credential& AuthenticationChallengeBase::proposedCredential() const 
{ 
    return m_proposedCredential; 
}

const ProtectionSpace& AuthenticationChallengeBase::protectionSpace() const 
{ 
    return m_protectionSpace; 
}

const ResourceResponse& AuthenticationChallengeBase::failureResponse() const 
{ 
    return m_failureResponse; 
}

const ResourceError& AuthenticationChallengeBase::error() const 
{ 
    return m_error; 
}

bool AuthenticationChallengeBase::isNull() const
{
    return m_isNull;
}

void AuthenticationChallengeBase::nullify()
{
    m_isNull = true;
}

bool AuthenticationChallengeBase::compare(const AuthenticationChallenge& a, const AuthenticationChallenge& b)
{
    if (a.isNull() && b.isNull())
        return true;

    if (a.isNull() || b.isNull())
        return false;
        
    if (a.protectionSpace() != b.protectionSpace())
        return false;
        
    if (a.proposedCredential() != b.proposedCredential())
        return false;
        
    if (a.previousFailureCount() != b.previousFailureCount())
        return false;
        
    if (a.failureResponse() != b.failureResponse())
        return false;
        
    if (a.error() != b.error())
        return false;
        
    return AuthenticationChallenge::platformCompare(a, b);
}

}
