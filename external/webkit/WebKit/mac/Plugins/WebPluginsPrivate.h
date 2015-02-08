

// WebPluginWillPresentNativeUserInterfaceNotification is sent by plugins to notify the WebKit client
// application that the plugin will present a dialog or some other "native" user interface to the user.
// This is currently only needed by Dashboard, which must hide itself when a plugin presents a dialog
// box (4318632).
extern NSString *WebPluginWillPresentNativeUserInterfaceNotification;
