

#ifndef WebSecurityOrigin_h
#define WebSecurityOrigin_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class SecurityOrigin; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebSecurityOriginPrivate;
class WebString;

class WebSecurityOrigin {
public:
    ~WebSecurityOrigin() { reset(); }

    WebSecurityOrigin() : m_private(0) { }
    WebSecurityOrigin(const WebSecurityOrigin& s) : m_private(0) { assign(s); }
    WebSecurityOrigin& operator=(const WebSecurityOrigin& s)
    {
        assign(s);
        return *this;
    }

    // FIXME: This should return a WebSecurityOrigin, not a pointer to one.
    WEBKIT_API static WebSecurityOrigin* createFromDatabaseIdentifier(const WebString& databaseIdentifier);
    WEBKIT_API static WebSecurityOrigin createFromString(const WebString&);

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebSecurityOrigin&);

    bool isNull() const { return !m_private; }

    WEBKIT_API WebString protocol() const;
    WEBKIT_API WebString host() const;
    WEBKIT_API unsigned short port() const;

    // The empty WebSecurityOrigin is the least privileged WebSecurityOrigin.
    WEBKIT_API bool isEmpty() const;

    // Returns a string representation of the WebSecurityOrigin.  The empty
    // WebSecurityOrigin is represented by "null".  The representation of a
    // non-empty WebSecurityOrigin resembles a standard URL.
    WEBKIT_API WebString toString() const;

    // Returns a string representation of this WebSecurityOrigin that can
    // be used as a file.  Should be used in storage APIs only.
    WEBKIT_API WebString databaseIdentifier();

#if WEBKIT_IMPLEMENTATION
    WebSecurityOrigin(const WTF::PassRefPtr<WebCore::SecurityOrigin>&);
    WebSecurityOrigin& operator=(const WTF::PassRefPtr<WebCore::SecurityOrigin>&);
    operator WTF::PassRefPtr<WebCore::SecurityOrigin>() const;
#endif

private:
    void assign(WebSecurityOriginPrivate*);
    WebSecurityOriginPrivate* m_private;
};

} // namespace WebKit

#endif
