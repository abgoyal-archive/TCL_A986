

#ifndef NotificationPresenter_h
#define NotificationPresenter_h

#include <wtf/PassRefPtr.h>
#include "VoidCallback.h"

#if ENABLE(NOTIFICATIONS)

namespace WebCore {

    class Document;
    class Notification;
    class KURL;
    class SecurityOrigin;
    class String;

    class NotificationPresenter {

    public:
        enum Permission {
            PermissionAllowed, // User has allowed notifications
            PermissionNotAllowed, // User has not yet allowed
            PermissionDenied // User has explicitly denied permission
        };

        virtual ~NotificationPresenter() {}

        // Requests that a notification be shown.
        virtual bool show(Notification*) = 0;

        // Requests that a notification that has already been shown be canceled.
        virtual void cancel(Notification*) = 0;

        // Informs the presenter that a Notification object has been destroyed
        // (such as by a page transition).  The presenter may continue showing
        // the notification, but must not attempt to call the event handlers.
        virtual void notificationObjectDestroyed(Notification*) = 0;

        // Requests user permission to show desktop notifications from a particular
        // origin. The callback parameter should be run when the user has
        // made a decision.
        virtual void requestPermission(SecurityOrigin*, PassRefPtr<VoidCallback>) = 0;

        // Checks the current level of permission for the specified URL. If the
        // URL is a document (as opposed to a worker or other ScriptExecutionContext),
        // |document| will also be provided.
        virtual Permission checkPermission(const KURL&, Document*) = 0;
    };

} // namespace WebCore

#endif // ENABLE(NOTIFICATIONS)

#endif // NotificationPresenter_h
