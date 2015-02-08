

#ifndef WebData_h
#define WebData_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class SharedBuffer; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebDataPrivate;

// A container for raw bytes.  It is inexpensive to copy a WebData object.
//
// WARNING: It is not safe to pass a WebData across threads!!!
//
class WebData {
public:
    ~WebData() { reset(); }

    WebData() : m_private(0) { }

    WebData(const char* data, size_t size) : m_private(0)
    {
        assign(data, size);
    }

    template <int N>
    WebData(const char (&data)[N]) : m_private(0)
    {
        assign(data, N - 1);
    }

    WebData(const WebData& d) : m_private(0) { assign(d); }

    WebData& operator=(const WebData& d)
    {
        assign(d);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebData&);
    WEBKIT_API void assign(const char* data, size_t size);

    WEBKIT_API size_t size() const;
    WEBKIT_API const char* data() const;

    bool isEmpty() const { return !size(); }
    bool isNull() const { return !m_private; }

#if WEBKIT_IMPLEMENTATION
    WebData(const WTF::PassRefPtr<WebCore::SharedBuffer>&);
    WebData& operator=(const WTF::PassRefPtr<WebCore::SharedBuffer>&);
    operator WTF::PassRefPtr<WebCore::SharedBuffer>() const;
#else
    template <class C>
    WebData(const C& c) : m_private(0)
    {
        assign(c.data(), c.size());
    }

    template <class C>
    WebData& operator=(const C& c)
    {
        assign(c.data(), c.size());
        return *this;
    }
#endif

private:
    void assign(WebDataPrivate*);
    WebDataPrivate* m_private;
};

} // namespace WebKit

#endif
