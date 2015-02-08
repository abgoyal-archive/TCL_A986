

#include "config.h"

#if ENABLE(WORKERS)

#include "WorkerLocation.h"

#include "PlatformString.h"

namespace WebCore {

String WorkerLocation::href() const
{
    return m_url.hasPath() ? m_url.prettyURL() : m_url.prettyURL() + "/";
}

String WorkerLocation::protocol() const
{
    return m_url.protocol() + ":";
}

String WorkerLocation::host() const
{
    return m_url.port() ? m_url.host() + ":" + String::number((static_cast<int>(m_url.port()))) : m_url.host();
}

String WorkerLocation::hostname() const
{
    return m_url.host();
}

String WorkerLocation::port() const
{
    return m_url.port() ? String::number(static_cast<int>(m_url.port())) : "";
}

String WorkerLocation::pathname() const
{
    return m_url.path().isEmpty() ? "/" : m_url.path();
}

String WorkerLocation::search() const
{
    return m_url.query().isEmpty() ? "" : "?" + m_url.query();
}

String WorkerLocation::hash() const
{
    return m_url.fragmentIdentifier().isEmpty() ? "" : "#" + m_url.fragmentIdentifier();
}

String WorkerLocation::toString() const
{
    return m_url.hasPath() ? m_url.prettyURL() : m_url.prettyURL() + "/";
}


} // namespace WebCore

#endif // ENABLE(WORKERS)
