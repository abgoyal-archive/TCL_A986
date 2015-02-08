

#include "config.h"
#include "JSHTMLElement.h"

#include "Document.h"
#include "HTMLFormElement.h"

namespace WebCore {

using namespace JSC;

void JSHTMLElement::pushEventHandlerScope(ExecState* exec, ScopeChain& scope) const
{
    HTMLElement* element = impl();

    // The document is put on first, fall back to searching it only after the element and form.
    scope.push(asObject(toJS(exec, globalObject(), element->ownerDocument())));

    // The form is next, searched before the document, but after the element itself.
    if (HTMLFormElement* form = element->form())
        scope.push(asObject(toJS(exec, globalObject(), form)));

    // The element is on top, searched first.
    scope.push(asObject(toJS(exec, globalObject(), element)));
}

} // namespace WebCore
