

#include "config.h"

#if ENABLE(WML)
#include "WMLAElement.h"

#include "DNS.h"
#include "Event.h"
#include "EventHandler.h"
#include "EventNames.h"
#include "Frame.h"
#include "FrameLoader.h"
#include "HTMLNames.h"
#include "KeyboardEvent.h"
#include "MappedAttribute.h"
#include "MouseEvent.h"
#include "RenderBox.h"
#include "WMLNames.h"

namespace WebCore {

using namespace WMLNames;

WMLAElement::WMLAElement(const QualifiedName& tagName, Document* doc)
    : WMLElement(tagName, doc)
{
}

void WMLAElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == HTMLNames::hrefAttr) {
        bool wasLink = isLink();
        setIsLink(!attr->isNull());
        if (wasLink != isLink())
            setNeedsStyleRecalc();
        if (isLink() && document()->isDNSPrefetchEnabled()) {
            String value = attr->value();
            if (protocolIs(value, "http") || protocolIs(value, "https") || value.startsWith("//"))
                prefetchDNS(document()->completeURL(value).host());
        }
    } else if (attr->name() == HTMLNames::nameAttr
               || attr->name() == HTMLNames::titleAttr
               || attr->name() == HTMLNames::relAttr) {
        // Do nothing.
    } else
        WMLElement::parseMappedAttribute(attr);
}

bool WMLAElement::supportsFocus() const
{
    return isLink() || WMLElement::supportsFocus();
}

bool WMLAElement::isMouseFocusable() const
{
    return false;
}

bool WMLAElement::isKeyboardFocusable(KeyboardEvent* event) const
{
    if (!isFocusable())
        return false;
    
    if (!document()->frame())
        return false;

    if (!document()->frame()->eventHandler()->tabsToLinks(event))
        return false;

    if (!renderer() || !renderer()->isBoxModelObject())
        return false;

    // Before calling absoluteRects, check for the common case where the renderer
    // is non-empty, since this is a faster check and almost always returns true.
    RenderBoxModelObject* box = toRenderBoxModelObject(renderer());
    if (!box->borderBoundingBox().isEmpty())
        return true;

    Vector<IntRect> rects;
    FloatPoint absPos = renderer()->localToAbsolute();
    renderer()->absoluteRects(rects, absPos.x(), absPos.y());
    size_t n = rects.size();
    for (size_t i = 0; i < n; ++i)
        if (!rects[i].isEmpty())
            return true;

    return false;
}

void WMLAElement::defaultEventHandler(Event* event)
{
    if (isLink() && (event->type() == eventNames().clickEvent || (event->type() == eventNames().keydownEvent && focused()))) {
        MouseEvent* e = 0;
        if (event->type() == eventNames().clickEvent && event->isMouseEvent())
            e = static_cast<MouseEvent*>(event);

        KeyboardEvent* k = 0;
        if (event->type() == eventNames().keydownEvent && event->isKeyboardEvent())
            k = static_cast<KeyboardEvent*>(event);

        if (e && e->button() == RightButton) {
            WMLElement::defaultEventHandler(event);
            return;
        }

        if (k) {
            if (k->keyIdentifier() != "Enter") {
                WMLElement::defaultEventHandler(event);
                return;
            }

            event->setDefaultHandled();
            dispatchSimulatedClick(event);
            return;
        }
 
        if (!event->defaultPrevented() && document()->frame()) {
            String url = document()->completeURL(deprecatedParseURL(getAttribute(HTMLNames::hrefAttr)));
            document()->frame()->loader()->urlSelected(url, target(), event, false, false, true, SendReferrer);
        }

        event->setDefaultHandled();
    }

    WMLElement::defaultEventHandler(event);
}

void WMLAElement::accessKeyAction(bool sendToAnyElement)
{
    // send the mouse button events if the caller specified sendToAnyElement
    dispatchSimulatedClick(0, sendToAnyElement);
}

bool WMLAElement::isURLAttribute(Attribute *attr) const
{
    return attr->name() == HTMLNames::hrefAttr;
}

String WMLAElement::target() const
{
    return getAttribute(HTMLNames::targetAttr);
}

}

#endif
