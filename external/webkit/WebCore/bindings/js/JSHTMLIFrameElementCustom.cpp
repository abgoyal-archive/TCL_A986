

#include "config.h"
#include "JSHTMLIFrameElement.h"

#include "CSSHelper.h"
#include "Document.h"
#include "HTMLIFrameElement.h"
#include "HTMLNames.h"
#include "JSDOMBinding.h"

using namespace JSC;

namespace WebCore {

using namespace HTMLNames;

void JSHTMLIFrameElement::setSrc(ExecState* exec, JSValue value)
{
    HTMLIFrameElement* imp = static_cast<HTMLIFrameElement*>(impl());

    String srcValue = valueToStringWithNullCheck(exec, value);

    if (protocolIsJavaScript(deprecatedParseURL(srcValue))) {
        Document* contentDocument = imp->contentDocument();
        if (contentDocument && !checkNodeSecurity(exec, contentDocument))
            return;
    }

    imp->setAttribute(srcAttr, srcValue);
}

} // namespace WebCore
