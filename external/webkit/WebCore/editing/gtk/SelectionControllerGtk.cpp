
 
#include "config.h"
#include "SelectionController.h"

#include "AccessibilityObjectWrapperAtk.h"
#include "AXObjectCache.h"
#include "Frame.h"

#include <gtk/gtk.h>

namespace WebCore {

void SelectionController::notifyAccessibilityForSelectionChange()
{
    if (AXObjectCache::accessibilityEnabled() && m_selection.start().isNotNull() && m_selection.end().isNotNull()) {
        RenderObject* focusedNode = m_selection.end().node()->renderer();
        AccessibilityObject* accessibilityObject = m_frame->document()->axObjectCache()->getOrCreate(focusedNode);
        int offset;
        // Always report the events w.r.t. the non-linked unignored parent. (i.e. ignoreLinks == true)
        AccessibilityObject* object = objectAndOffsetUnignored(accessibilityObject, offset, true);
        AtkObject* wrapper = object->wrapper();
        if (ATK_IS_TEXT(wrapper)) {
            g_signal_emit_by_name(wrapper, "text-caret-moved", offset);
            if (m_selection.isRange())
                g_signal_emit_by_name(wrapper, "text-selection-changed");
        }
    }
}

} // namespace WebCore
