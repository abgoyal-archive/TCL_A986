

#include "config.h"
#include "ScriptController.h"

#include "Bridge.h"
#include "DOMWindow.h"
#include "PluginView.h"
#include "qt_instance.h"
#include "runtime_root.h"

#include <QWidget>

namespace WebCore {

PassRefPtr<JSC::Bindings::Instance> ScriptController::createScriptInstanceForWidget(WebCore::Widget* widget)
{
    if (widget->isPluginView()) {
        PluginView* pluginView = static_cast<PluginView*>(widget);
        return pluginView->bindingInstance();
    }

    QWidget* platformWidget = widget->platformWidget();
    if (!platformWidget)
        return 0;
    return JSC::Bindings::QtInstance::getQtInstance(platformWidget, bindingRootObject(), QScriptEngine::QtOwnership);
}

}
// vim: ts=4 sw=4 et
