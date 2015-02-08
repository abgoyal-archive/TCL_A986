

#ifndef NotificationCenter_h
#define NotificationCenter_h

#include "Notification.h"
#include "NotificationContents.h"
#include "WorkerThread.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

#if ENABLE(NOTIFICATIONS)

namespace WebCore {

    class ScriptExecutionContext;

    class NotificationCenter : public RefCounted<NotificationCenter>, public ActiveDOMObject { 
    public:
        static PassRefPtr<NotificationCenter> create(ScriptExecutionContext* context, NotificationPresenter* presenter) { return adoptRef(new NotificationCenter(context, presenter)); }

        Notification* createHTMLNotification(const String& URI, ExceptionCode& ec)
        {
            if (!presenter()) {
                ec = INVALID_STATE_ERR;
                return 0;
            }
            return Notification::create(KURL(ParsedURLString, URI), context(), ec, presenter());
        }

        Notification* createNotification(const String& iconURI, const String& title, const String& body, ExceptionCode& ec)
        {
            if (!presenter()) {
                ec = INVALID_STATE_ERR;
                return 0;
            }
            NotificationContents contents(iconURI, title, body);
            return Notification::create(contents, context(), ec, presenter());
        }

        ScriptExecutionContext* context() const { return m_scriptExecutionContext; }
        NotificationPresenter* presenter() const { return m_notificationPresenter; }

        int checkPermission();
        void requestPermission(PassRefPtr<VoidCallback> callback);

        void disconnectFrame() { m_notificationPresenter = 0; }

    private:
        NotificationCenter(ScriptExecutionContext*, NotificationPresenter*);

        ScriptExecutionContext* m_scriptExecutionContext;
        NotificationPresenter* m_notificationPresenter;
    };

} // namespace WebCore

#endif // ENABLE(NOTIFICATIONS)

#endif // NotificationCenter_h
