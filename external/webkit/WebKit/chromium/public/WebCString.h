

#ifndef WebCString_h
#define WebCString_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class CString; }
#else
#include <string>
#endif

namespace WebKit {

class WebCStringPrivate;
class WebString;

// A single-byte string container with unspecified encoding.  It is
// inexpensive to copy a WebCString object.
//
// WARNING: It is not safe to pass a WebCString across threads!!!
//
class WebCString {
public:
    ~WebCString() { reset(); }

    WebCString() : m_private(0) { }

    WebCString(const char* data, size_t len) : m_private(0)
    {
        assign(data, len);
    }

    WebCString(const WebCString& s) : m_private(0) { assign(s); }

    WebCString& operator=(const WebCString& s)
    {
        assign(s);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebCString&);
    WEBKIT_API void assign(const char* data, size_t len);

    WEBKIT_API size_t length() const;
    WEBKIT_API const char* data() const;

    bool isEmpty() const { return !length(); }
    bool isNull() const { return !m_private; }

    WEBKIT_API WebString utf16() const;

    WEBKIT_API static WebCString fromUTF16(const WebUChar* data, size_t length);
    WEBKIT_API static WebCString fromUTF16(const WebUChar* data);

#if WEBKIT_IMPLEMENTATION
    WebCString(const WebCore::CString&);
    WebCString& operator=(const WebCore::CString&);
    operator WebCore::CString() const;
#else
    WebCString(const std::string& s) : m_private(0)
    {
        assign(s.data(), s.length());
    }

    WebCString& operator=(const std::string& s)
    {
        assign(s.data(), s.length());
        return *this;
    }

    operator std::string() const
    {
        size_t len = length();
        return len ? std::string(data(), len) : std::string();
    }

    template <class UTF16String>
    static WebCString fromUTF16(const UTF16String& s)
    {
        return fromUTF16(s.data(), s.length());
    }
#endif

private:
    void assign(WebCStringPrivate*);
    WebCStringPrivate* m_private;
};

} // namespace WebKit

#endif
