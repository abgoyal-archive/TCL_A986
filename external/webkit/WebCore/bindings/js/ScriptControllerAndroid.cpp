

#include "config.h"
#include "ScriptController.h"

#include "PluginView.h"
#include "Bridge.h"

namespace WebCore {

#ifdef ANDROID_PLUGINS
PassRefPtr<JSC::Bindings::Instance> ScriptController::createScriptInstanceForWidget(Widget* widget)
{
    if (!widget->isPluginView())
        return 0;

    return static_cast<PluginView*>(widget)->bindingInstance();
}
#endif

} // namespace WebCore
