

#ifndef NotificationContents_h
#define NotificationContents_h

#if ENABLE(NOTIFICATIONS)

namespace WebCore {

    class NotificationContents { 
    public:
        NotificationContents() {}
        NotificationContents(const String& iconUrl, const String& title, const String& body) 
            : m_icon(iconUrl)
            , m_title(title)
            , m_body(body) {}

        String icon() const { return m_icon; }
        String title() const { return m_title; }
        String body() const { return m_body; }

    private:
        String m_icon;
        String m_title;
        String m_body;
    };

} // namespace WebCore

#endif // ENABLE(NOTIFICATIONS)

#endif // NotificationContents_h
