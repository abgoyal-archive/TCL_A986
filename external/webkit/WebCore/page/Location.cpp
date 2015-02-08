

#include "config.h"
#include "Location.h"

#include "Frame.h"
#include "FrameLoader.h"
#include "KURL.h"
#include "PlatformString.h"

namespace WebCore {

Location::Location(Frame* frame)
    : m_frame(frame)
{
}

void Location::disconnectFrame()
{
    m_frame = 0;
}

inline const KURL& Location::url() const
{
    ASSERT(m_frame);

    const KURL& url = m_frame->loader()->url();
    if (!url.isValid())
        return blankURL();  // Use "about:blank" while the page is still loading (before we have a frame).

    return url;
}

String Location::href() const
{
    if (!m_frame)
        return String();

    const KURL& url = this->url();
    return url.hasPath() ? url.prettyURL() : url.prettyURL() + "/";
}

String Location::protocol() const
{
    if (!m_frame)
        return String();

    return url().protocol() + ":";
}

String Location::host() const
{
    if (!m_frame)
        return String();

    // Note: this is the IE spec. The NS spec swaps the two, it says
    // "The hostname property is the concatenation of the host and port properties, separated by a colon."
    const KURL& url = this->url();
    return url.port() ? url.host() + ":" + String::number((static_cast<int>(url.port()))) : url.host();
}

String Location::hostname() const
{
    if (!m_frame)
        return String();

    return url().host();
}

String Location::port() const
{
    if (!m_frame)
        return String();

    const KURL& url = this->url();
    return url.port() ? String::number(static_cast<int>(url.port())) : "";
}

String Location::pathname() const
{
    if (!m_frame)
        return String();

    const KURL& url = this->url();
    return url.path().isEmpty() ? "/" : url.path();
}

String Location::search() const
{
    if (!m_frame)
        return String();

    const KURL& url = this->url();
    return url.query().isEmpty() ? "" : "?" + url.query();
}

String Location::hash() const
{
    if (!m_frame)
        return String();

    const String& fragmentIdentifier = this->url().fragmentIdentifier();
    return fragmentIdentifier.isEmpty() ? "" : "#" + fragmentIdentifier;
}

String Location::toString() const
{
    if (!m_frame)
        return String();

    const KURL& url = this->url();
    return url.hasPath() ? url.prettyURL() : url.prettyURL() + "/";
}

} // namespace WebCore
