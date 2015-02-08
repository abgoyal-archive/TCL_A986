

#ifndef WebScriptController_h
#define WebScriptController_h

#include "WebCommon.h"

namespace v8 {
class Extension;
}

namespace WebKit {

class WebString;

class WebScriptController {
public:
    // Registers a v8 extension to be available on webpages. The three forms
    // offer various restrictions on what types of contexts the extension is
    // loaded into. If a scheme is provided, only pages whose URL has the given
    // scheme will match. If extensionGroup is provided, the extension will only
    // be loaded into scripts run via WebFrame::ExecuteInNewWorld with the
    // matching group.
    // Will only affect v8 contexts initialized after this call. Takes ownership
    // of the v8::Extension object passed.
    WEBKIT_API static void registerExtension(v8::Extension*);
    WEBKIT_API static void registerExtension(v8::Extension*,
                                             const WebString& schemeRestriction);
    WEBKIT_API static void registerExtension(v8::Extension*, int extensionGroup);

    // Enables special settings which are only applicable if V8 is executed
    // in the single thread which must be the main thread.
    // FIXME: make a try to dynamically detect when this condition is broken
    // and automatically switch off single thread mode.
    WEBKIT_API static void enableV8SingleThreadMode();

    // Process any pending JavaScript console messages.
    WEBKIT_API static void flushConsoleMessages();

private:
    WebScriptController();
};

} // namespace WebKit

#endif
