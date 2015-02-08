

#include "config.h"
#include "JSInspectorFrontendHost.h"

#if ENABLE(INSPECTOR)

#include "ContextMenuItem.h"
#include "InspectorController.h"
#include "InspectorFrontendHost.h"
#include "JSEvent.h"
#include "MouseEvent.h"
#include <runtime/JSArray.h>
#include <runtime/JSLock.h>
#include <runtime/JSObject.h>
#include <wtf/Vector.h>

using namespace JSC;

namespace WebCore {

JSValue JSInspectorFrontendHost::showContextMenu(ExecState* execState, const ArgList& args)
{
    if (args.size() < 2)
        return jsUndefined();

    Event* event = toEvent(args.at(0));

    JSArray* array = asArray(args.at(1));
    Vector<ContextMenuItem*> items;

    for (size_t i = 0; i < array->length(); ++i) {
        JSObject* item = asObject(array->getIndex(i));
        JSValue label = item->get(execState, Identifier(execState, "label"));
        JSValue id = item->get(execState, Identifier(execState, "id"));
        if (label.isUndefined() || id.isUndefined())
            items.append(new ContextMenuItem(SeparatorType, ContextMenuItemTagNoAction, String()));
        else {
            ContextMenuAction typedId = static_cast<ContextMenuAction>(ContextMenuItemBaseCustomTag + id.toInt32(execState));
            items.append(new ContextMenuItem(ActionType, typedId, label.toString(execState)));
        }
    }

    impl()->showContextMenu(event, items);
    return jsUndefined();
}

} // namespace WebCore

#endif // ENABLE(INSPECTOR)
