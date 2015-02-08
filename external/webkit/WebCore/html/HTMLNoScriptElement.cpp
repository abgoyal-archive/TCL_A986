

#include "config.h"

#if ENABLE(XHTMLMP)
#include "HTMLNoScriptElement.h"

#include "CSSStyleSelector.h"
#include "HTMLNames.h"
#include "RenderObject.h"

namespace WebCore {

using namespace HTMLNames;

HTMLNoScriptElement::HTMLNoScriptElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
{
    ASSERT(hasTagName(noscriptTag));
}

HTMLNoScriptElement::~HTMLNoScriptElement()
{
}

bool HTMLNoScriptElement::checkDTD(const Node* newChild)
{
    return newChild->isTextNode() || inBlockTagList(newChild); 
}

void HTMLNoScriptElement::attach()
{
    HTMLElement::attach();

    // If no need to process <noscript>, we hide it by setting display:none temporarily
    if (!document()->shouldProcessNoscriptElement()) {
        if (renderer() && renderer()->style())
            renderer()->style()->setDisplay(NONE);
        setNeedsStyleRecalc();
    }
}

void HTMLNoScriptElement::recalcStyle(StyleChange change)
{
    if (!document()->shouldProcessNoscriptElement() || !renderer() || !renderer()->style())
        return;

    // If <noscript> needs processing, we make it visiable here, including its visible children
    RefPtr<RenderStyle> style = renderer()->style();
    if (style->display() == NONE) {
        style->setDisplay(INLINE);

        // Create renderers for its children
        if (hasChildNodes()) {
            for (Node* n = firstChild(); n; n = n->traverseNextNode(this))
                if (!n->renderer())
                    n->createRendererIfNeeded();
        }
    }
}

bool HTMLNoScriptElement::childShouldCreateRenderer(Node* child) const
{
    return document()->shouldProcessNoscriptElement();
}

}
#endif
