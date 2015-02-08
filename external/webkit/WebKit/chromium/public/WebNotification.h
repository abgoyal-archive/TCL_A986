

#ifndef WebNotification_h
#define WebNotification_h

#include "WebCommon.h"

#if WEBKIT_IMPLEMENTATION
namespace WebCore { class Notification; }
namespace WTF { template <typename T> class PassRefPtr; }
#endif

namespace WebKit {

class WebNotificationPrivate;
class WebURL;
class WebString;

// Represents access to a desktop notification.
class WebNotification {
public:
    WebNotification() : m_private(0) { }
    WebNotification(const WebNotification& other) : m_private(0) { assign(other); }

    ~WebNotification() { reset(); }

    WEBKIT_API void reset();
    WEBKIT_API void assign(const WebNotification&);

    WebNotification& operator=(const WebNotification& other)
    {
        assign(other);
        return *this;
    }

    // Operators required to put WebNotification in an ordered map.
    bool equals(const WebNotification& other) const { return m_private == other.m_private; }
    bool lessThan(const WebNotification& other) const;

    // Is the notification HTML vs. icon-title-text?
    WEBKIT_API bool isHTML() const;

    // If HTML, the URL which contains the contents of the notification.
    WEBKIT_API WebURL url() const;

    // If not HTML, the parameters for the icon-title-text notification.
    WEBKIT_API WebString icon() const;
    WEBKIT_API WebString title() const;
    WEBKIT_API WebString body() const;

    // Called to indicate the notification has been displayed.
    WEBKIT_API void dispatchDisplayEvent();

    // Called to indicate an error has occurred with this notification.
    WEBKIT_API void dispatchErrorEvent(const WebString& errorMessage);

    // Called to indicate the notification has been closed.  If it was
    // closed by the user (as opposed to automatically by the system),
    // the byUser parameter will be true.
    WEBKIT_API void dispatchCloseEvent(bool byUser);

#if WEBKIT_IMPLEMENTATION
    WebNotification(const WTF::PassRefPtr<WebCore::Notification>&);
    WebNotification& operator=(const WTF::PassRefPtr<WebCore::Notification>&);
    operator WTF::PassRefPtr<WebCore::Notification>() const;
#endif

private:
    void assign(WebNotificationPrivate*);
    WebNotificationPrivate* m_private;
};

inline bool operator==(const WebNotification& a, const WebNotification& b)
{
    return a.equals(b);
}

inline bool operator!=(const WebNotification& a, const WebNotification& b)
{
    return !a.equals(b);
}

inline bool operator<(const WebNotification& a, const WebNotification& b)
{
    return a.lessThan(b);
}

} // namespace WebKit

#endif
