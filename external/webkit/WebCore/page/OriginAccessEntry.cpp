

#include "config.h"
#include "OriginAccessEntry.h"

#include "SecurityOrigin.h"

namespace WebCore {
    
OriginAccessEntry::OriginAccessEntry(const String& protocol, const String& host, SubdomainSetting subdomainSetting)
    : m_protocol(protocol.lower())
    , m_host(host.lower())
    , m_subdomainSettings(subdomainSetting)
{
    ASSERT(m_protocol == "http" || m_protocol == "https");
    ASSERT(subdomainSetting == AllowSubdomains || subdomainSetting == DisallowSubdomains);

    // Assume that any host that ends with a digit is trying to be an IP address.
    m_hostIsIPAddress = !m_host.isEmpty() && isASCIIDigit(m_host[m_host.length() - 1]);
}

bool OriginAccessEntry::matchesOrigin(const SecurityOrigin& origin) const
{
    ASSERT(origin.host() == origin.host().lower());
    ASSERT(origin.protocol() == origin.protocol().lower());

    if (m_protocol != origin.protocol())
        return false;
    
    // Special case: Include subdomains and empty host means "all hosts, including ip addresses".
    if (m_subdomainSettings == AllowSubdomains && m_host.isEmpty())
        return true;
    
    // Exact match.
    if (m_host == origin.host())
        return true;
    
    // Otherwise we can only match if we're matching subdomains.
    if (m_subdomainSettings == DisallowSubdomains)
        return false;
    
    // Don't try to do subdomain matching on IP addresses.
    if (m_hostIsIPAddress)
        return false;
    
    // Match subdomains.
    if (origin.host().length() > m_host.length() && origin.host()[origin.host().length() - m_host.length() - 1] == '.' && origin.host().endsWith(m_host))
        return true;
    
    return false;
}
    
} // namespace WebCore
