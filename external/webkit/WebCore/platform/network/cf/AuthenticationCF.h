

#ifndef AuthenticationCF_h
#define AuthenticationCF_h

typedef struct _CFURLAuthChallenge* CFURLAuthChallengeRef;
typedef struct _CFURLCredential* CFURLCredentialRef;
typedef struct _CFURLProtectionSpace* CFURLProtectionSpaceRef;

namespace WebCore {

class AuthenticationChallenge;
class Credential;
class ProtectionSpace;

CFURLAuthChallengeRef createCF(const AuthenticationChallenge&);
CFURLCredentialRef createCF(const Credential&);
CFURLProtectionSpaceRef createCF(const ProtectionSpace&);

Credential core(CFURLCredentialRef);
ProtectionSpace core(CFURLProtectionSpaceRef);

}

#endif
