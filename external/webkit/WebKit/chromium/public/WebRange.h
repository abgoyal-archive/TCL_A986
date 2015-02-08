

#ifndef WebRange_h
#define WebRange_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Range; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebNode;
class WebRangePrivate;
class WebString;

// Provides readonly access to some properties of a DOM range.
class WebRange {
public:
    ~WebRange() { reset(); }

    WebRange() : m_private(0) { }
    WebRange(const WebRange& r) : m_private(0) { assign(r); }
    WebRange& operator=(const WebRange& r)
    {
        assign(r);
        return *this;
    }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebRange&);

    bool isNull() const { return !m_private; }

    WEBKIT_API int startOffset() const;
    WEBKIT_API int endOffset() const;
    WEBKIT_API WebNode startContainer(int& exceptionCode) const;
    WEBKIT_API WebNode endContainer(int& exceptionCode) const;

    WEBKIT_API WebString toHTMLText() const;
    WEBKIT_API WebString toPlainText() const;

#if WEBKIT_IMPLEMENTATION
    WebRange(const WTF::PassRefPtr<WebCore::Range>&);
    WebRange& operator=(const WTF::PassRefPtr<WebCore::Range>&);
    operator WTF::PassRefPtr<WebCore::Range>() const;
#endif

private:
    void assign(WebRangePrivate*);
    WebRangePrivate* m_private;
};

} // namespace WebKit

#endif
