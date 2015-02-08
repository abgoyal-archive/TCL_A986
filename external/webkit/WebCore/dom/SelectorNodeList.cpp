

#include "config.h"
#include "SelectorNodeList.h"

#include "CSSSelector.h"
#include "CSSSelectorList.h"
#include "CSSStyleSelector.h"
#include "Document.h"
#include "Element.h"
#include "HTMLNames.h"

namespace WebCore {

using namespace HTMLNames;

PassRefPtr<StaticNodeList> createSelectorNodeList(Node* rootNode, const CSSSelectorList& querySelectorList)
{
    Vector<RefPtr<Node> > nodes;
    Document* document = rootNode->document();
    CSSSelector* onlySelector = querySelectorList.hasOneSelector() ? querySelectorList.first() : 0;
    bool strictParsing = !document->inCompatMode();

    CSSStyleSelector::SelectorChecker selectorChecker(document, strictParsing);

    if (strictParsing && rootNode->inDocument() && onlySelector && onlySelector->m_match == CSSSelector::Id && !document->containsMultipleElementsWithId(onlySelector->m_value)) {
        Element* element = document->getElementById(onlySelector->m_value);
        if (element && (rootNode->isDocumentNode() || element->isDescendantOf(rootNode)) && selectorChecker.checkSelector(onlySelector, element))
            nodes.append(element);
    } else {
        for (Node* n = rootNode->firstChild(); n; n = n->traverseNextNode(rootNode)) {
            if (n->isElementNode()) {
                Element* element = static_cast<Element*>(n);
                for (CSSSelector* selector = querySelectorList.first(); selector; selector = CSSSelectorList::next(selector)) {
                    if (selectorChecker.checkSelector(selector, element)) {
                        nodes.append(n);
                        break;
                    }
                }
            }
        }
    }
    
    return StaticNodeList::adopt(nodes);
}

} // namespace WebCore
