

#include "config.h"
#include "ScriptController.h"

#include "Bridge.h"
#include "PluginView.h"

using namespace JSC::Bindings;

namespace WebCore {

PassRefPtr<JSC::Bindings::Instance> ScriptController::createScriptInstanceForWidget(Widget* widget)
{
    if (!widget->isPluginView())
        return 0;

    return static_cast<PluginView*>(widget)->bindingInstance();
}

} // namespace WebCore
