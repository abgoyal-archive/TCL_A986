

#include "config.h"
#include "V8StyleSheetList.h"

#include "HTMLStyleElement.h"
#include "StyleSheetList.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8StyleSheet.h"

namespace WebCore {

v8::Handle<v8::Value> V8StyleSheetList::namedPropertyGetter(v8::Local<v8::String> name, const v8::AccessorInfo& info)
{
    INC_STATS("DOM.StyleSheetList.NamedPropertyGetter");

    if (info.Holder()->HasRealNamedProperty(name))
        return notHandledByInterceptor();

    // Search style sheet.
    StyleSheetList* imp = V8StyleSheetList::toNative(info.Holder());
    HTMLStyleElement* item = imp->getNamedItem(toWebCoreString(name));
    if (!item)
        return notHandledByInterceptor();

    return toV8(item->sheet());
}

} // namespace WebCore
