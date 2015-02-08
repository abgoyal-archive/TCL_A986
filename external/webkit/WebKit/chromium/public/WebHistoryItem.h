

#ifndef WebHistoryItem_h
#define WebHistoryItem_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class HistoryItem; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebHistoryItemPrivate;
class WebHTTPBody;
class WebString;
struct WebPoint;
template <typename T> class WebVector;

// Represents a frame-level navigation entry in session history.  A
// WebHistoryItem is a node in a tree.
//
// Copying a WebHistoryItem is cheap.
//
class WebHistoryItem {
public:
    ~WebHistoryItem() { reset(); }

    WebHistoryItem() : m_private(0) { }
    WebHistoryItem(const WebHistoryItem& h) : m_private(0) { assign(h); }
    WebHistoryItem& operator=(const WebHistoryItem& h)
    {
        assign(h);
        return *this;
    }

    WEBKIT_API void initialize();
    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebHistoryItem&);

    bool isNull() const { return !m_private; }

    WEBKIT_API WebString urlString() const;
    WEBKIT_API void setURLString(const WebString&);

    WEBKIT_API WebString originalURLString() const;
    WEBKIT_API void setOriginalURLString(const WebString&);

    WEBKIT_API WebString referrer() const;
    WEBKIT_API void setReferrer(const WebString&);

    WEBKIT_API WebString target() const;
    WEBKIT_API void setTarget(const WebString&);

    WEBKIT_API WebString parent() const;
    WEBKIT_API void setParent(const WebString&);

    WEBKIT_API WebString title() const;
    WEBKIT_API void setTitle(const WebString&);

    WEBKIT_API WebString alternateTitle() const;
    WEBKIT_API void setAlternateTitle(const WebString&);

    WEBKIT_API double lastVisitedTime() const;
    WEBKIT_API void setLastVisitedTime(double);

    WEBKIT_API WebPoint scrollOffset() const;
    WEBKIT_API void setScrollOffset(const WebPoint&);

    WEBKIT_API bool isTargetItem() const;
    WEBKIT_API void setIsTargetItem(bool);

    WEBKIT_API int visitCount() const;
    WEBKIT_API void setVisitCount(int);

    WEBKIT_API WebVector<WebString> documentState() const;
    WEBKIT_API void setDocumentState(const WebVector<WebString>&);

    WEBKIT_API long long documentSequenceNumber() const;
    WEBKIT_API void setDocumentSequenceNumber(long long);

    WEBKIT_API WebString httpContentType() const;
    WEBKIT_API void setHTTPContentType(const WebString&);

    WEBKIT_API WebHTTPBody httpBody() const;
    WEBKIT_API void setHTTPBody(const WebHTTPBody&);

    WEBKIT_API WebVector<WebHistoryItem> children() const;
    WEBKIT_API void setChildren(const WebVector<WebHistoryItem>&);
    WEBKIT_API void appendToChildren(const WebHistoryItem&);

#if WEBKIT_IMPLEMENTATION
    WebHistoryItem(const WTF::PassRefPtr<WebCore::HistoryItem>&);
    WebHistoryItem& operator=(const WTF::PassRefPtr<WebCore::HistoryItem>&);
    operator WTF::PassRefPtr<WebCore::HistoryItem>() const;
#endif

private:
    void assign(WebHistoryItemPrivate*);
    void ensureMutable();
    WebHistoryItemPrivate* m_private;
};

} // namespace WebKit

#endif
