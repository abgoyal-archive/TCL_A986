

#include "config.h"
#include "JSHTMLFrameElement.h"

#include "CSSHelper.h"
#include "Document.h"
#include "HTMLFrameElement.h"
#include "HTMLNames.h"
#include "JSDOMBinding.h"

using namespace JSC;

namespace WebCore {

using namespace HTMLNames;

static inline bool allowSettingJavascriptURL(ExecState* exec, HTMLFrameElement* imp, const String& value)
{
    if (protocolIsJavaScript(deprecatedParseURL(value))) {
        Document* contentDocument = imp->contentDocument();
        if (contentDocument && !checkNodeSecurity(exec, contentDocument))
            return false;
    }
    return true;
}

void JSHTMLFrameElement::setSrc(ExecState* exec, JSValue value)
{
    HTMLFrameElement* imp = static_cast<HTMLFrameElement*>(impl());
    String srcValue = valueToStringWithNullCheck(exec, value);

    if (!allowSettingJavascriptURL(exec, imp, srcValue))
        return;

    imp->setAttribute(srcAttr, srcValue);
}

void JSHTMLFrameElement::setLocation(ExecState* exec, JSValue value)
{
    HTMLFrameElement* imp = static_cast<HTMLFrameElement*>(impl());
    String locationValue = valueToStringWithNullCheck(exec, value);

    if (!allowSettingJavascriptURL(exec, imp, locationValue))
        return;

    imp->setLocation(locationValue);
}

} // namespace WebCore
