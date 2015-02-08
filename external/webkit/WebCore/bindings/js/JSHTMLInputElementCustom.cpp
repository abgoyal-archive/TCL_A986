
 
#include "config.h"
#include "JSHTMLInputElement.h"

#include "Document.h"
#include "HTMLInputElement.h"
#include "Settings.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

static bool needsGmailQuirk(HTMLInputElement* input)
{
    Document* document = input->document();

    const KURL& url = document->url();
    if (url.host() != "mail.google.com")
        return false;

    // As with other site-specific quirks, allow website developers to turn this off.
    // In theory, this allows website developers to check if their fixes are effective.
    Settings* settings = document->settings();
    if (!settings)
        return false;
    if (!settings->needsSiteSpecificQuirks())
        return false;

    return true;
}

JSValue JSHTMLInputElement::type(ExecState* exec) const
{
    HTMLInputElement* input = static_cast<HTMLInputElement*>(impl());
    const AtomicString& type = input->type();

    DEFINE_STATIC_LOCAL(const AtomicString, url, ("url"));
    DEFINE_STATIC_LOCAL(const AtomicString, text, ("text"));

    if (type == url && needsGmailQuirk(input))
        return jsString(exec, text);
    return jsString(exec, type);
}

JSValue JSHTMLInputElement::selectionStart(ExecState* exec) const
{
    HTMLInputElement* input = static_cast<HTMLInputElement*>(impl());
    if (!input->canHaveSelection())
        return throwError(exec, TypeError);

    return jsNumber(exec, input->selectionStart());
}

void JSHTMLInputElement::setSelectionStart(ExecState* exec, JSValue value)
{
    HTMLInputElement* input = static_cast<HTMLInputElement*>(impl());
    if (!input->canHaveSelection())
        throwError(exec, TypeError);

    input->setSelectionStart(value.toInt32(exec));
}

JSValue JSHTMLInputElement::selectionEnd(ExecState* exec) const
{
    HTMLInputElement* input = static_cast<HTMLInputElement*>(impl());
    if (!input->canHaveSelection())
        return throwError(exec, TypeError);

    return jsNumber(exec, input->selectionEnd());
}

void JSHTMLInputElement::setSelectionEnd(ExecState* exec, JSValue value)
{
    HTMLInputElement* input = static_cast<HTMLInputElement*>(impl());
    if (!input->canHaveSelection())
        throwError(exec, TypeError);

    input->setSelectionEnd(value.toInt32(exec));
}

JSValue JSHTMLInputElement::setSelectionRange(ExecState* exec, const ArgList& args)
{
    HTMLInputElement* input = static_cast<HTMLInputElement*>(impl());
    if (!input->canHaveSelection())
        return throwError(exec, TypeError);

    int start = args.at(0).toInt32(exec);
    int end = args.at(1).toInt32(exec);

    input->setSelectionRange(start, end);
    return jsUndefined();
}

} // namespace WebCore
