

#ifndef ANDROID_WEBKIT_WEBICONDATABASE_H
#define ANDROID_WEBKIT_WEBICONDATABASE_H

#include "IconDatabaseClient.h"
#include "utils/threads.h"
#include "wtf/Vector.h"

#include <jni.h>

namespace WebCore {
    class Image;
    class String;
}

namespace android {

    class WebIconDatabaseClient {
    public:
        virtual ~WebIconDatabaseClient() {}
        virtual void didAddIconForPageUrl(const WebCore::String& pageUrl) = 0;
    };

    class WebIconDatabase : public WebCore::IconDatabaseClient {
    public:
        WebIconDatabase() : mDeliveryRequested(false) {}
        // IconDatabaseClient method
        virtual void dispatchDidAddIconForPageURL(const WebCore::String& pageURL);

        static void RegisterForIconNotification(WebIconDatabaseClient* client);
        static void UnregisterForIconNotification(WebIconDatabaseClient* client);
        static void DeliverNotifications(void*);

    private:
        // Deliver all the icon notifications
        void deliverNotifications();

        // List of clients.
        Vector<WebIconDatabaseClient*> mClients;

        // Queue of page urls that have received an icon.
        Vector<WebCore::String> mNotifications;
        android::Mutex          mNotificationsMutex;
        // Flag to indicate that we have requested a delivery of notifications.
        bool                    mDeliveryRequested;
    };

    jobject webcoreImageToJavaBitmap(JNIEnv* env, WebCore::Image* icon);

};

#endif
