

#include "config.h"
#include "HTMLMarqueeElement.h"

#include "CSSPropertyNames.h"
#include "CSSValueKeywords.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderLayer.h"
#include "RenderMarquee.h"

namespace WebCore {

using namespace HTMLNames;

// WinIE uses 60ms as the minimum delay by default.
const int defaultMinimumDelay = 60;

HTMLMarqueeElement::HTMLMarqueeElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
    , ActiveDOMObject(doc, this)
    , m_minimumDelay(defaultMinimumDelay)
{
    ASSERT(hasTagName(marqueeTag));
}

bool HTMLMarqueeElement::mapToEntry(const QualifiedName& attrName, MappedAttributeEntry& result) const
{
    if (attrName == widthAttr ||
        attrName == heightAttr ||
        attrName == bgcolorAttr ||
        attrName == vspaceAttr ||
        attrName == hspaceAttr ||
        attrName == scrollamountAttr ||
        attrName == scrolldelayAttr ||
        attrName == loopAttr ||
        attrName == behaviorAttr ||
        attrName == directionAttr) {
        result = eUniversal;
        return false;
    }

    return HTMLElement::mapToEntry(attrName, result);
}

void HTMLMarqueeElement::parseMappedAttribute(MappedAttribute *attr)
{
    if (attr->name() == widthAttr) {
        if (!attr->value().isEmpty())
            addCSSLength(attr, CSSPropertyWidth, attr->value());
    } else if (attr->name() == heightAttr) {
        if (!attr->value().isEmpty())
            addCSSLength(attr, CSSPropertyHeight, attr->value());
    } else if (attr->name() == bgcolorAttr) {
        if (!attr->value().isEmpty())
            addCSSColor(attr, CSSPropertyBackgroundColor, attr->value());
    } else if (attr->name() == vspaceAttr) {
        if (!attr->value().isEmpty()) {
            addCSSLength(attr, CSSPropertyMarginTop, attr->value());
            addCSSLength(attr, CSSPropertyMarginBottom, attr->value());
        }
    } else if (attr->name() == hspaceAttr) {
        if (!attr->value().isEmpty()) {
            addCSSLength(attr, CSSPropertyMarginLeft, attr->value());
            addCSSLength(attr, CSSPropertyMarginRight, attr->value());
        }
    } else if (attr->name() == scrollamountAttr) {
        if (!attr->value().isEmpty())
            addCSSLength(attr, CSSPropertyWebkitMarqueeIncrement, attr->value());
    } else if (attr->name() == scrolldelayAttr) {
        if (!attr->value().isEmpty())
            addCSSLength(attr, CSSPropertyWebkitMarqueeSpeed, attr->value());
    } else if (attr->name() == loopAttr) {
        if (!attr->value().isEmpty()) {
            if (attr->value() == "-1" || equalIgnoringCase(attr->value(), "infinite"))
                addCSSProperty(attr, CSSPropertyWebkitMarqueeRepetition, CSSValueInfinite);
            else
                addCSSLength(attr, CSSPropertyWebkitMarqueeRepetition, attr->value());
        }
    } else if (attr->name() == behaviorAttr) {
        if (!attr->value().isEmpty())
            addCSSProperty(attr, CSSPropertyWebkitMarqueeStyle, attr->value());
    } else if (attr->name() == directionAttr) {
        if (!attr->value().isEmpty())
            addCSSProperty(attr, CSSPropertyWebkitMarqueeDirection, attr->value());
    } else if (attr->name() == truespeedAttr)
        m_minimumDelay = !attr->isEmpty() ? 0 : defaultMinimumDelay;
    else
        HTMLElement::parseMappedAttribute(attr);
}

void HTMLMarqueeElement::start()
{
    if (RenderMarquee* marqueeRenderer = renderMarquee())
        marqueeRenderer->start();
}

void HTMLMarqueeElement::stop()
{
    if (RenderMarquee* marqueeRenderer = renderMarquee())
        marqueeRenderer->stop();
}

bool HTMLMarqueeElement::canSuspend() const
{
    return true;
}

void HTMLMarqueeElement::suspend()
{
    if (RenderMarquee* marqueeRenderer = renderMarquee())
        marqueeRenderer->suspend();
}

void HTMLMarqueeElement::resume()
{
    if (RenderMarquee* marqueeRenderer = renderMarquee())
        marqueeRenderer->updateMarqueePosition();
}

RenderMarquee* HTMLMarqueeElement::renderMarquee() const
{
    if (renderer() && renderer()->hasLayer())
        return renderBoxModelObject()->layer()->marquee();
    return 0;
}

} // namespace WebCore
