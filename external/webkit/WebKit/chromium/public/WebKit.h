

#ifndef WebKit_h
#define WebKit_h

#include "WebCommon.h"

namespace WebKit {

class WebKitClient;

// Must be called on the thread that will be the main WebKit thread before
// using any other WebKit APIs.  The provided WebKitClient must be non-null
// and must remain valid until the current thread calls shutdown.
WEBKIT_API void initialize(WebKitClient*);

// Once shutdown, the WebKitClient passed to initialize will no longer be
// accessed.  No other WebKit objects should be in use when this function
// is called.  Any background threads created by WebKit are promised to be
// terminated by the time this function returns.
WEBKIT_API void shutdown();

// Returns the WebKitClient instance passed to initialize.
WEBKIT_API WebKitClient* webKitClient();

// Alters the rendering of content to conform to a fixed set of rules.
WEBKIT_API void setLayoutTestMode(bool);
WEBKIT_API bool layoutTestMode();

// Enables the named log channel.  See WebCore/platform/Logging.h for details.
WEBKIT_API void enableLogChannel(const char*);

// Purge the plugin list cache. If |reloadPages| is true, any pages
// containing plugins will be reloaded after refreshing the plugin list.
WEBKIT_API void resetPluginCache(bool reloadPages);

} // namespace WebKit

#endif
