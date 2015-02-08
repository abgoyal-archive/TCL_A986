
#ifndef AuthenticationMac_h
#define AuthenticationMac_h

#ifdef __OBJC__

@class NSURLAuthenticationChallenge;
@class NSURLCredential;
@class NSURLProtectionSpace;

namespace WebCore {

class AuthenticationChallenge;
class Credential;
class ProtectionSpace;

NSURLAuthenticationChallenge *mac(const AuthenticationChallenge&);
NSURLProtectionSpace *mac(const ProtectionSpace&);
NSURLCredential *mac(const Credential&);

AuthenticationChallenge core(NSURLAuthenticationChallenge *);
ProtectionSpace core(NSURLProtectionSpace *);
Credential core(NSURLCredential *);

}
#endif

#endif
