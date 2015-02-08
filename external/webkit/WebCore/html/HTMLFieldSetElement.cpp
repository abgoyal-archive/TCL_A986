

#include "config.h"
#include "HTMLFieldSetElement.h"

#include "HTMLNames.h"
#include "RenderFieldset.h"
#include <wtf/StdLibExtras.h>

namespace WebCore {

using namespace HTMLNames;

HTMLFieldSetElement::HTMLFieldSetElement(const QualifiedName& tagName, Document *doc, HTMLFormElement *f)
    : HTMLFormControlElement(tagName, doc, f)
{
    ASSERT(hasTagName(fieldsetTag));
}

HTMLFieldSetElement::~HTMLFieldSetElement()
{
}

bool HTMLFieldSetElement::checkDTD(const Node* newChild)
{
    return newChild->hasTagName(legendTag) || HTMLElement::checkDTD(newChild);
}

bool HTMLFieldSetElement::supportsFocus() const
{
    return HTMLElement::supportsFocus();
}

const AtomicString& HTMLFieldSetElement::formControlType() const
{
    DEFINE_STATIC_LOCAL(const AtomicString, fieldset, ("fieldset"));
    return fieldset;
}

RenderObject* HTMLFieldSetElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderFieldset(this);
}

} // namespace
