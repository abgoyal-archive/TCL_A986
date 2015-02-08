

#ifndef Notification_h  
#define Notification_h

#include "ActiveDOMObject.h"
#include "AtomicStringHash.h"
#include "Event.h"
#include "EventListener.h"
#include "EventNames.h"
#include "EventTarget.h"
#include "ExceptionCode.h"
#include "KURL.h"
#include "NotificationPresenter.h"
#include "NotificationContents.h"
#include "RegisteredEventListener.h"
#include <wtf/OwnPtr.h>
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

#if ENABLE(NOTIFICATIONS)
namespace WebCore {

    class WorkerContext;

    class Notification : public RefCounted<Notification>, public ActiveDOMObject, public EventTarget { 
    public:
        static Notification* create(const String& url, ScriptExecutionContext* context, ExceptionCode& ec, NotificationPresenter* provider) { return new Notification(url, context, ec, provider); }
        static Notification* create(const NotificationContents& contents, ScriptExecutionContext* context, ExceptionCode& ec, NotificationPresenter* provider) { return new Notification(contents, context, ec, provider); }
        
        virtual ~Notification();

        void show();
        void cancel();
    
        bool isHTML() { return m_isHTML; }
        KURL url() { return m_notificationURL; }
        NotificationContents& contents() { return m_contents; }

        DEFINE_ATTRIBUTE_EVENT_LISTENER(display);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(error);
        DEFINE_ATTRIBUTE_EVENT_LISTENER(close);
    
        using RefCounted<Notification>::ref;
        using RefCounted<Notification>::deref;
    
        // EventTarget interface
        virtual ScriptExecutionContext* scriptExecutionContext() const { return ActiveDOMObject::scriptExecutionContext(); }
        virtual Notification* toNotification() { return this; }

    private:
        Notification(const String& url, ScriptExecutionContext* context, ExceptionCode& ec, NotificationPresenter* provider);
        Notification(const NotificationContents& fields, ScriptExecutionContext* context, ExceptionCode& ec, NotificationPresenter* provider);

        // EventTarget interface
        virtual void refEventTarget() { ref(); }
        virtual void derefEventTarget() { deref(); }
        virtual EventTargetData* eventTargetData();
        virtual EventTargetData* ensureEventTargetData();

        bool m_isHTML;
        KURL m_notificationURL;
        NotificationContents m_contents;

        bool m_isShowing;

        NotificationPresenter* m_presenter;
        
        EventTargetData m_eventTargetData;
    };

} // namespace WebCore

#endif // ENABLE(NOTIFICATIONS)

#endif // Notifications_h
