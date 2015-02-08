

#include "config.h"
#include "DataGridColumnList.h"

#include "Document.h"
#include "V8Binding.h"
#include "V8Proxy.h"

#if ENABLE(DATAGRID)

namespace WebCore {

NAMED_PROPERTY_GETTER(DataGridColumnList)
{
    INC_STATS("DataGridColumnList.NamedPropertyGetter");
    // Search the prototype chain first.
    v8::Handle<v8::Value> value = info.Holder()->GetRealNamedPropertyInPrototypeChain(name);
    if (!value.IsEmpty())
        return value;
    
    // Then look for IDL defined properties on the object itself.
    if (info.Holder()->HasRealNamedCallbackProperty(name))
        return notHandledByInterceptor();
    
    // Finally, look up a column by name.
    DataGridColumnList* imp = V8DataGridColumnList::toNative(info.Holder());
    DataGridColumn* result = imp->itemWithName(toWebCoreString(name));
    if (!result)
        return notHandledByInterceptor();
    return toV8(result);
}
    
} // namespace WebCore

#endif // ENABLE(DATAGRID)
