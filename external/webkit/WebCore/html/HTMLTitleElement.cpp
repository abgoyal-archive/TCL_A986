
#include "config.h"
#include "HTMLTitleElement.h"

#include "Document.h"
#include "HTMLNames.h"
#include "Text.h"

namespace WebCore {

using namespace HTMLNames;

HTMLTitleElement::HTMLTitleElement(const QualifiedName& tagName, Document* doc)
    : HTMLElement(tagName, doc)
    , m_title("")
{
    ASSERT(hasTagName(titleTag));
}

HTMLTitleElement::~HTMLTitleElement()
{
}

void HTMLTitleElement::insertedIntoDocument()
{
    HTMLElement::insertedIntoDocument();
    document()->setTitle(m_title, this);
}

void HTMLTitleElement::removedFromDocument()
{
    HTMLElement::removedFromDocument();
    document()->removeTitle(this);
}

void HTMLTitleElement::childrenChanged(bool changedByParser, Node* beforeChange, Node* afterChange, int childCountDelta)
{
    m_title = "";
    for (Node* c = firstChild(); c != 0; c = c->nextSibling())
        if (c->nodeType() == TEXT_NODE || c->nodeType() == CDATA_SECTION_NODE)
            m_title += c->nodeValue();
    if (inDocument())
        document()->setTitle(m_title, this);
    HTMLElement::childrenChanged(changedByParser, beforeChange, afterChange, childCountDelta);
}

String HTMLTitleElement::text() const
{
    String val = "";
    
    for (Node *n = firstChild(); n; n = n->nextSibling()) {
        if (n->isTextNode())
            val += static_cast<Text*>(n)->data();
    }
    
    return val;
}

void HTMLTitleElement::setText(const String &value)
{
    ExceptionCode ec = 0;
    int numChildren = childNodeCount();
    
    if (numChildren == 1 && firstChild()->isTextNode())
        static_cast<Text*>(firstChild())->setData(value, ec);
    else {  
        if (numChildren > 0)
            removeChildren();
    
        appendChild(document()->createTextNode(value.impl()), ec);
    }
}

}
