

#include "config.h"
#include "HTMLTableRowsCollection.h"

#include "HTMLNames.h"
#include "HTMLTableElement.h"
#include "HTMLTableRowElement.h"

namespace WebCore {

using namespace HTMLNames;

static bool isInHead(Element* row)
{
    return row->parent() && static_cast<Element*>(row->parent())->hasLocalName(theadTag);
}

static bool isInBody(Element* row)
{
    return row->parent() && static_cast<Element*>(row->parent())->hasLocalName(tbodyTag);
}

static bool isInFoot(Element* row)
{
    return row->parent() && static_cast<Element*>(row->parent())->hasLocalName(tfootTag);
}

HTMLTableRowElement* HTMLTableRowsCollection::rowAfter(HTMLTableElement* table, HTMLTableRowElement* previous)
{
    Node* child = 0;

    // Start by looking for the next row in this section.
    // Continue only if there is none.
    if (previous && previous->parent() != table) {
        for (child = previous->nextSibling(); child; child = child->nextSibling()) {
            if (child->hasTagName(trTag))
                return static_cast<HTMLTableRowElement*>(child);
        }
    }

    // If still looking at head sections, find the first row in the next head section.
    if (!previous)
        child = table->firstChild();
    else if (isInHead(previous))
        child = previous->parent()->nextSibling();
    for (; child; child = child->nextSibling()) {
        if (child->hasTagName(theadTag)) {
            for (Node* grandchild = child->firstChild(); grandchild; grandchild = grandchild->nextSibling()) {
                if (grandchild->hasTagName(trTag))
                    return static_cast<HTMLTableRowElement*>(grandchild);
            }
        }
    }

    // If still looking at top level and bodies, find the next row in top level or the first in the next body section.
    if (!previous || isInHead(previous))
        child = table->firstChild();
    else if (previous->parent() == table)
        child = previous->nextSibling();
    else if (isInBody(previous))
        child = previous->parent()->nextSibling();
    for (; child; child = child->nextSibling()) {
        if (child->hasTagName(trTag))
            return static_cast<HTMLTableRowElement*>(child);
        if (child->hasTagName(tbodyTag)) {
            for (Node* grandchild = child->firstChild(); grandchild; grandchild = grandchild->nextSibling()) {
                if (grandchild->hasTagName(trTag))
                    return static_cast<HTMLTableRowElement*>(grandchild);
            }
        }
    }

    // Find the first row in the next foot section.
    if (!previous || !isInFoot(previous))
        child = table->firstChild();
    else
        child = previous->parent()->nextSibling();
    for (; child; child = child->nextSibling()) {
        if (child->hasTagName(tfootTag)) {
            for (Node* grandchild = child->firstChild(); grandchild; grandchild = grandchild->nextSibling()) {
                if (grandchild->hasTagName(trTag))
                    return static_cast<HTMLTableRowElement*>(grandchild);
            }
        }
    }

    return 0;
}

HTMLTableRowElement* HTMLTableRowsCollection::lastRow(HTMLTableElement* table)
{
    for (Node* child = table->lastChild(); child; child = child->previousSibling()) {
        if (child->hasTagName(tfootTag)) {
            for (Node* grandchild = child->lastChild(); grandchild; grandchild = grandchild->previousSibling()) {
                if (grandchild->hasTagName(trTag))
                    return static_cast<HTMLTableRowElement*>(grandchild);
            }
        }
    }

    for (Node* child = table->lastChild(); child; child = child->previousSibling()) {
        if (child->hasTagName(trTag))
            return static_cast<HTMLTableRowElement*>(child);
        if (child->hasTagName(tbodyTag)) {
            for (Node* grandchild = child->lastChild(); grandchild; grandchild = grandchild->previousSibling()) {
                if (grandchild->hasTagName(trTag))
                    return static_cast<HTMLTableRowElement*>(grandchild);
            }
        }
    }

    for (Node* child = table->lastChild(); child; child = child->previousSibling()) {
        if (child->hasTagName(theadTag)) {
            for (Node* grandchild = child->lastChild(); grandchild; grandchild = grandchild->previousSibling()) {
                if (grandchild->hasTagName(trTag))
                    return static_cast<HTMLTableRowElement*>(grandchild);
            }
        }
    }

    return 0;
}

HTMLTableRowsCollection::HTMLTableRowsCollection(PassRefPtr<HTMLTableElement> table)
    : HTMLCollection(table, OtherCollection, 0)
{
}

PassRefPtr<HTMLTableRowsCollection> HTMLTableRowsCollection::create(PassRefPtr<HTMLTableElement> table)
{
    return adoptRef(new HTMLTableRowsCollection(table));
}

Element* HTMLTableRowsCollection::itemAfter(Element* previous) const
{
    ASSERT(!previous || previous->hasLocalName(trTag));
    return rowAfter(static_cast<HTMLTableElement*>(base()), static_cast<HTMLTableRowElement*>(previous));
}

}
