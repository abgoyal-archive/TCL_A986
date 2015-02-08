

#ifndef WebNotificationPermissionCallback_h
#define WebNotificationPermissionCallback_h

namespace WebKit {

// Callback object used with WebNotificationPresenter.
class WebNotificationPermissionCallback {
public:
    // Method to be invoked when the async permission request involving
    // this object has been completed.
    virtual void permissionRequestComplete() = 0;

protected:
    // This object is to be destroyed after the callback is invoked;
    // it should not be directly destroyed.
    ~WebNotificationPermissionCallback() {}
};

} // namespace WebKit

#endif
