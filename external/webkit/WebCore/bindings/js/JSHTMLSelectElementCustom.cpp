

#include "config.h"
#include "JSHTMLSelectElementCustom.h"

#include "ExceptionCode.h"
#include "HTMLNames.h"
#include "HTMLOptionElement.h"
#include "HTMLSelectElement.h"
#include "JSHTMLOptionElement.h"

namespace WebCore {

using namespace JSC;
using namespace HTMLNames;

JSValue JSHTMLSelectElement::remove(ExecState* exec, const ArgList& args)
{
    HTMLSelectElement& select = *static_cast<HTMLSelectElement*>(impl());

    // we support both options index and options objects
    HTMLElement* element = toHTMLElement(args.at(0));
    if (element && element->hasTagName(optionTag))
        select.remove(static_cast<HTMLOptionElement*>(element)->index());
    else
        select.remove(args.at(0).toInt32(exec));

    return jsUndefined();
}

void selectIndexSetter(HTMLSelectElement* select, JSC::ExecState* exec, unsigned index, JSC::JSValue value)
{
    if (value.isUndefinedOrNull())
        select->remove(index);
    else {
        ExceptionCode ec = 0;
        HTMLOptionElement* option = toHTMLOptionElement(value);
        if (!option)
            ec = TYPE_MISMATCH_ERR;
        else
            select->setOption(index, option, ec);
        setDOMException(exec, ec);
    }
}

void JSHTMLSelectElement::indexSetter(JSC::ExecState* exec, unsigned index, JSC::JSValue value)
{
    selectIndexSetter(static_cast<HTMLSelectElement*>(impl()), exec, index, value);
}

}
