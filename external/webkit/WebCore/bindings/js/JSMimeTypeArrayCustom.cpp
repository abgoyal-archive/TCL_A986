

#include "config.h"
#include "JSMimeTypeArray.h"

#include "AtomicString.h"
#include "JSMimeType.h"
#include "MimeTypeArray.h"

namespace WebCore {

using namespace JSC;

bool JSMimeTypeArray::canGetItemsForName(ExecState*, MimeTypeArray* mimeTypeArray, const Identifier& propertyName)
{
    return mimeTypeArray->canGetItemsForName(propertyName);
}

JSValue JSMimeTypeArray::nameGetter(ExecState* exec, const Identifier& propertyName, const PropertySlot& slot)
{
    JSMimeTypeArray* thisObj = static_cast<JSMimeTypeArray*>(asObject(slot.slotBase()));
    return toJS(exec, thisObj->impl()->namedItem(propertyName));
}

} // namespace WebCore
