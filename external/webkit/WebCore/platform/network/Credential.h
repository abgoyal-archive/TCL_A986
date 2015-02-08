
#ifndef Credential_h
#define Credential_h

#include "PlatformString.h"

#define CERTIFICATE_CREDENTIALS_SUPPORTED ((PLATFORM(MAC) || PLATFORM(IPHONE)) && !defined(BUILDING_ON_TIGER) && !defined(BUILDING_ON_LEOPARD))

#if CERTIFICATE_CREDENTIALS_SUPPORTED
#include <Security/SecBase.h>
#include <wtf/RetainPtr.h>
#endif

namespace WebCore {

enum CredentialPersistence {
    CredentialPersistenceNone,
    CredentialPersistenceForSession,
    CredentialPersistencePermanent
};

#if CERTIFICATE_CREDENTIALS_SUPPORTED
enum CredentialType {
    CredentialTypePassword,
    CredentialTypeClientCertificate
};
#endif

class Credential {

public:
    Credential();
    Credential(const String& user, const String& password, CredentialPersistence);
    Credential(const Credential& original, CredentialPersistence);
#if CERTIFICATE_CREDENTIALS_SUPPORTED
    Credential(SecIdentityRef identity, CFArrayRef certificates, CredentialPersistence);
#endif
    
    bool isEmpty() const;
    
    const String& user() const;
    const String& password() const;
    bool hasPassword() const;
    CredentialPersistence persistence() const;
    
#if CERTIFICATE_CREDENTIALS_SUPPORTED
    SecIdentityRef identity() const;
    CFArrayRef certificates() const;
    const CredentialType type() const;
#endif    
    
private:
    String m_user;
    String m_password;
    CredentialPersistence m_persistence;
#if CERTIFICATE_CREDENTIALS_SUPPORTED
    RetainPtr<SecIdentityRef> m_identity;
    RetainPtr<CFArrayRef> m_certificates;
    CredentialType m_type;
#endif
};

bool operator==(const Credential& a, const Credential& b);
inline bool operator!=(const Credential& a, const Credential& b) { return !(a == b); }
    
};
#endif
