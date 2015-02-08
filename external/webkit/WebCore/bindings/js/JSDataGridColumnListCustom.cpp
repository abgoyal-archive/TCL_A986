

#include "config.h"

#if ENABLE(DATAGRID)

#include "JSDataGridColumnList.h"

#include "AtomicString.h"
#include "DataGridColumn.h"
#include "DataGridColumnList.h"
#include "JSDataGridColumn.h"

using namespace JSC;

namespace WebCore {

bool JSDataGridColumnList::canGetItemsForName(ExecState*, DataGridColumnList* impl, const Identifier& propertyName)
{
    return impl->itemWithName(propertyName);
}

JSValue JSDataGridColumnList::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSDataGridColumnList* thisObj = static_cast<JSDataGridColumnList*>(asObject(slot.slotBase()));
    return toJS(exec, thisObj->globalObject(), thisObj->impl()->itemWithName(propertyName));
}

} // namespace WebCore

#endif
