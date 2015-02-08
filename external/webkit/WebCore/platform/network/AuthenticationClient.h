

#ifndef AuthenticationClient_h
#define AuthenticationClient_h

namespace WebCore {

class AuthenticationChallenge;
class Credential;

class AuthenticationClient {
public:
    virtual void receivedCredential(const AuthenticationChallenge&, const Credential&) = 0;
    virtual void receivedRequestToContinueWithoutCredential(const AuthenticationChallenge&) = 0;
    virtual void receivedCancellation(const AuthenticationChallenge&) = 0;

    void ref() { refAuthenticationClient(); }
    void deref() { derefAuthenticationClient(); }

protected:
    virtual ~AuthenticationClient() { }

private:
    virtual void refAuthenticationClient() = 0;
    virtual void derefAuthenticationClient() = 0;
};

}

#endif
