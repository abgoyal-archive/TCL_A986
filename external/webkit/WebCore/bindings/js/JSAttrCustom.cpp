

#include "config.h"
#include "JSAttr.h"

#include "CSSHelper.h"
#include "Document.h"
#include "HTMLFrameElementBase.h"
#include "HTMLNames.h"

using namespace JSC;

namespace WebCore {

using namespace HTMLNames;

void JSAttr::setValue(ExecState* exec, JSValue value)
{
    Attr* imp = static_cast<Attr*>(impl());
    String attrValue = valueToStringWithNullCheck(exec, value);

    Element* ownerElement = imp->ownerElement();
    if (ownerElement && (ownerElement->hasTagName(iframeTag) || ownerElement->hasTagName(frameTag))) {
        if (equalIgnoringCase(imp->name(), "src") && protocolIsJavaScript(deprecatedParseURL(attrValue))) {
            Document* contentDocument = static_cast<HTMLFrameElementBase*>(ownerElement)->contentDocument();
            if (contentDocument && !checkNodeSecurity(exec, contentDocument))
                return;
        }
    }

    ExceptionCode ec = 0;
    imp->setValue(attrValue, ec);
    setDOMException(exec, ec);
}

void JSAttr::markChildren(MarkStack& markStack)
{
    Base::markChildren(markStack);

    // Mark the element so that this will work to access the attribute even if the last
    // other reference goes away.
    if (Element* element = impl()->ownerElement())
        markDOMNodeWrapper(markStack, element->document(), element);
}

} // namespace WebCore
