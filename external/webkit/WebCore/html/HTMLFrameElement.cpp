

#include "config.h"
#include "HTMLFrameElement.h"

#include "Frame.h"
#include "HTMLFrameSetElement.h"
#include "HTMLNames.h"
#include "MappedAttribute.h"
#include "RenderFrame.h"

namespace WebCore {

using namespace HTMLNames;

inline HTMLFrameElement::HTMLFrameElement(const QualifiedName& tagName, Document* document)
    : HTMLFrameElementBase(tagName, document)
    , m_frameBorder(true)
    , m_frameBorderSet(false)
    , m_noResize(false)
{
    ASSERT(hasTagName(frameTag));
}

PassRefPtr<HTMLFrameElement> HTMLFrameElement::create(const QualifiedName& tagName, Document* document)
{
    return adoptRef(new HTMLFrameElement(tagName, document));
}

bool HTMLFrameElement::rendererIsNeeded(RenderStyle*)
{
    // For compatibility, frames render even when display: none is set.
    return isURLAllowed();
}

RenderObject* HTMLFrameElement::createRenderer(RenderArena* arena, RenderStyle*)
{
    return new (arena) RenderFrame(this);
}

static inline HTMLFrameSetElement* containingFrameSetElement(Node* node)
{
    while ((node = node->parentNode())) {
        if (node->hasTagName(framesetTag))
            return static_cast<HTMLFrameSetElement*>(node);
    }
    return 0;
}

void HTMLFrameElement::attach()
{
    HTMLFrameElementBase::attach();
    
    if (HTMLFrameSetElement* frameSetElement = containingFrameSetElement(this)) {
        if (!m_frameBorderSet)
            m_frameBorder = frameSetElement->hasFrameBorder();
        if (!m_noResize)
            m_noResize = frameSetElement->noResize();
    }
}

void HTMLFrameElement::parseMappedAttribute(MappedAttribute* attr)
{
    if (attr->name() == frameborderAttr) {
        m_frameBorder = attr->value().toInt();
        m_frameBorderSet = !attr->isNull();
        // FIXME: If we are already attached, this has no effect.
    } else if (attr->name() == noresizeAttr) {
        m_noResize = true;
        // FIXME: If we are already attached, this has no effect.
        // FIXME: Since this does not check attr->isNull(), it can
        // never reset m_noResize to false if the attribute is removed.
        // FIXME: There seems to be no code that looks at this
        // value and prevents resizing.
    } else
        HTMLFrameElementBase::parseMappedAttribute(attr);
}

void HTMLFrameElement::setNoResize(bool noResize)
{
    setAttribute(noresizeAttr, noResize ? "" : 0);
}

} // namespace WebCore
