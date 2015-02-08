
 
#ifndef UserScript_h
#define UserScript_h

#include "KURL.h"
#include "UserScriptTypes.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassOwnPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class UserScript : public Noncopyable {
public:
    UserScript(const String& source, const KURL& url,
               PassOwnPtr<Vector<String> > whitelist, PassOwnPtr<Vector<String> > blacklist,
               UserScriptInjectionTime injectionTime)
        : m_source(source)
        , m_url(url)
        , m_whitelist(whitelist)
        , m_blacklist(blacklist)
        , m_injectionTime(injectionTime)
    {
    }

    const String& source() const { return m_source; }
    const KURL& url() const { return m_url; }
    const Vector<String>* whitelist() const { return m_whitelist.get(); }
    const Vector<String>* blacklist() const { return m_blacklist.get(); }
    UserScriptInjectionTime injectionTime() const { return m_injectionTime; }
    
private:
    String m_source;
    KURL m_url;
    OwnPtr<Vector<String> > m_whitelist;
    OwnPtr<Vector<String> > m_blacklist;
    UserScriptInjectionTime m_injectionTime;
};

} // namespace WebCore
 
#endif // UserScript_h
