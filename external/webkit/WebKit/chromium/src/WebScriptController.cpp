

#include "config.h"
#include "WebScriptController.h"

#include "V8Binding.h"
#include "V8DOMMap.h"
#include "V8Proxy.h"

#include "WebString.h"

using namespace WebCore;

namespace WebKit {

void WebScriptController::registerExtension(v8::Extension* extension)
{
    V8Proxy::registerExtension(extension, WebString());
}

void WebScriptController::registerExtension(v8::Extension* extension,
                                            const WebString& schemeRestriction)
{
    V8Proxy::registerExtension(extension, schemeRestriction);
}

void WebScriptController::registerExtension(v8::Extension* extension,
                                            int extensionGroup)
{
    V8Proxy::registerExtension(extension, extensionGroup);
}

void WebScriptController::enableV8SingleThreadMode()
{
    enableStringImplCache();
    enableFasterDOMStoreAccess();
}

void WebScriptController::flushConsoleMessages()
{
    WebCore::V8Proxy::processConsoleMessages();
}

} // namespace WebKit
