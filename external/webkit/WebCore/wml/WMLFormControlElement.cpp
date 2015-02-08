

#include "config.h"

#if ENABLE(WML)
#include "WMLFormControlElement.h"

#include "RenderBox.h"
#include "RenderObject.h"
#include "RenderStyle.h"

namespace WebCore {

WMLFormControlElement::WMLFormControlElement(const QualifiedName& tagName, Document* document)
    : WMLElement(tagName, document)
    , m_valueMatchesRenderer(false)
{
}

WMLFormControlElement::~WMLFormControlElement()
{
}

bool WMLFormControlElement::supportsFocus() const
{
    return true;
}

bool WMLFormControlElement::isFocusable() const
{
    if (!renderer() || !renderer()->isBox())
        return false;

    if (toRenderBox(renderer())->size().isEmpty())
        return false;
    
    return WMLElement::isFocusable();
}
    

void WMLFormControlElement::attach()
{
    ASSERT(!attached());
    WMLElement::attach();

    // The call to updateFromElement() needs to go after the call through
    // to the base class's attach() because that can sometimes do a close
    // on the renderer.
    if (renderer())
        renderer()->updateFromElement();
}

void WMLFormControlElement::recalcStyle(StyleChange change)
{
    WMLElement::recalcStyle(change);

    if (renderer())
        renderer()->updateFromElement();
}

}

#endif
