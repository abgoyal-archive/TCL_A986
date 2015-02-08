

#include "config.h"
#include "V8InspectorFrontendHost.h"

#include "InspectorController.h"
#include "InspectorFrontendHost.h"

#include "V8Binding.h"
#include "V8Proxy.h"

namespace WebCore {

v8::Handle<v8::Value> V8InspectorFrontendHost::showContextMenuCallback(const v8::Arguments& args)
{
    return v8::Undefined();
}

} // namespace WebCore
