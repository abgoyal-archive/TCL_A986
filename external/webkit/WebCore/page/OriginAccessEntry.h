

#ifndef CrossOriginAccess_h
#define CrossOriginAccess_h

#include "PlatformString.h"

namespace WebCore {

class SecurityOrigin;

class OriginAccessEntry {
public:
    enum SubdomainSetting {
        AllowSubdomains,
        DisallowSubdomains
    };

    // If host is empty string and SubdomainSetting is AllowSubdomains, the entry will match all domains in the specified protocol.
    OriginAccessEntry(const String& protocol, const String& host, SubdomainSetting);
    bool matchesOrigin(const SecurityOrigin&) const;
    
private:
    String m_protocol;
    String m_host;
    SubdomainSetting m_subdomainSettings;
    bool m_hostIsIPAddress;
    
};

} // namespace WebCore

#endif // CrossOriginAccess_h
