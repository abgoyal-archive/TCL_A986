

#ifndef SessionCredentialStorage_h
#define SessionCredentialStorage_h

namespace WebCore {

class Credential;
class KURL;
class ProtectionSpace;

class CredentialStorage {
public:
    // WebCore session credential storage.
    static void set(const Credential&, const ProtectionSpace&, const KURL&);
    static Credential get(const ProtectionSpace&);

    // OS persistent storage.
    static Credential getFromPersistentStorage(const ProtectionSpace&);

    // These methods work for authentication schemes that support sending credentials without waiting for a request. E.g., for HTTP Basic authentication scheme
    // a client should assume that all paths at or deeper than the depth of a known protected resource share are within the same protection space.
    static bool set(const Credential&, const KURL&); // Returns true if the URL corresponds to a known protection space, so credentials could be updated.
    static Credential get(const KURL&);
};

} // namespace WebCore

#endif // SessionCredentialStorage_h
