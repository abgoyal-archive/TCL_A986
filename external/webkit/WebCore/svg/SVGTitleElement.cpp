

#include "config.h"
#if ENABLE(SVG)
#include "SVGTitleElement.h"

#include "Document.h"

namespace WebCore {

SVGTitleElement::SVGTitleElement(const QualifiedName& tagName, Document* doc)
    : SVGStyledElement(tagName, doc)
{
}

void SVGTitleElement::insertedIntoDocument()
{
    SVGStyledElement::insertedIntoDocument();
    if (firstChild())
        document()->setTitle(textContent(), this);
}

void SVGTitleElement::removedFromDocument()
{
    SVGElement::removedFromDocument();
    document()->removeTitle(this);
}

void SVGTitleElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    SVGElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);
    if (inDocument())
        document()->setTitle(textContent(), this);
}

}

// vim:ts=4:noet
#endif // ENABLE(SVG)
