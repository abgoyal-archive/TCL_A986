
 
#ifndef UserContentURLPattern_h
#define UserContentURLPattern_h

#include "PlatformString.h"
#include <wtf/Vector.h>

namespace WebCore {

class KURL;

class UserContentURLPattern {
public:
    UserContentURLPattern(const String& pattern)
    : m_matchSubdomains(false)
    {
        m_invalid = !parse(pattern);
    }

    bool matches(const KURL&) const;

    const String& scheme() const { return m_scheme; }
    const String& host() const { return m_host; }
    const String& path() const { return m_path; }

    bool matchSubdomains() const { return m_matchSubdomains; }
    
    static bool matchesPatterns(const KURL&, const Vector<String>* whitelist, const Vector<String>* blacklist);

private:
    bool parse(const String& pattern);

    bool matchesHost(const KURL&) const;
    bool matchesPath(const KURL&) const;

    bool m_invalid;

    String m_scheme;
    String m_host;
    String m_path;

    bool m_matchSubdomains;
};


} // namespace WebCore

#endif // UserContentURLPattern_h
