
 
#ifndef UserStyleSheet_h
#define UserStyleSheet_h

#include "KURL.h"
#include "UserStyleSheetTypes.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class UserStyleSheet : public Noncopyable {
public:
    UserStyleSheet(const String& source, const KURL& url,
                   PassOwnPtr<Vector<String> > whitelist, PassOwnPtr<Vector<String> > blacklist)
        : m_source(source)
        , m_url(url)
        , m_whitelist(whitelist)
        , m_blacklist(blacklist)
    {
    }

    const String& source() const { return m_source; }
    const KURL& url() const { return m_url; }
    const Vector<String>* whitelist() const { return m_whitelist.get(); }
    const Vector<String>* blacklist() const { return m_blacklist.get(); }

private:
    String m_source;
    KURL m_url;
    OwnPtr<Vector<String> > m_whitelist;
    OwnPtr<Vector<String> > m_blacklist;
};

} // namespace WebCore
 
#endif // UserStyleSheet_h
