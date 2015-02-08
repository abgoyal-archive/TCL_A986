

#ifndef WebURL_h
#define WebURL_h

#include "WebCString.h"
#include <googleurl/src/url_parse.h>

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class KURL; }
#else
#include <googleurl/src/gurl.h>
#endif

namespace WebKit {

class WebURL {
public:
    ~WebURL()
    {
    }

    WebURL() : m_isValid(false)
    {
    }

    WebURL(const WebCString& spec, const url_parse::Parsed& parsed, bool isValid)
        : m_spec(spec)
        , m_parsed(parsed)
        , m_isValid(isValid)
    {
    }

    WebURL(const WebURL& s)
        : m_spec(s.m_spec)
        , m_parsed(s.m_parsed)
        , m_isValid(s.m_isValid)
    {
    }

    WebURL& operator=(const WebURL& s)
    {
        m_spec = s.m_spec;
        m_parsed = s.m_parsed;
        m_isValid = s.m_isValid;
        return *this;
    }

    void assign(const WebCString& spec, const url_parse::Parsed& parsed, bool isValid)
    {
        m_spec = spec;
        m_parsed = parsed;
        m_isValid = isValid;
    }

    const WebCString& spec() const
    {
        return m_spec;
    }

    const url_parse::Parsed& parsed() const
    {
        return m_parsed;
    }

    bool isValid() const
    {
        return m_isValid;
    }

    bool isEmpty() const
    {
        return m_spec.isEmpty();
    }

    bool isNull() const
    {
        return m_spec.isEmpty();
    }

#if WEBKIT_IMPLEMENTATION
    WebURL(const WebCore::KURL&);
    WebURL& operator=(const WebCore::KURL&);
    operator WebCore::KURL() const;
#else
    WebURL(const GURL& g)
        : m_spec(g.possibly_invalid_spec())
        , m_parsed(g.parsed_for_possibly_invalid_spec())
        , m_isValid(g.is_valid())
    {
    }

    WebURL& operator=(const GURL& g)
    {
        m_spec = g.possibly_invalid_spec();
        m_parsed = g.parsed_for_possibly_invalid_spec();
        m_isValid = g.is_valid();
        return *this;
    }

    operator GURL() const
    {
        return isNull() ? GURL() : GURL(m_spec.data(), m_spec.length(), m_parsed, m_isValid);
    }
#endif

private:
    WebCString m_spec;  // UTF-8 encoded
    url_parse::Parsed m_parsed;
    bool m_isValid;
};

} // namespace WebKit

#endif
