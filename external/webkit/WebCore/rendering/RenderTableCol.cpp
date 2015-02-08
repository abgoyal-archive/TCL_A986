

#include "config.h"
#include "RenderTableCol.h"

#include "CachedImage.h"
#include "HTMLNames.h"
#include "HTMLTableColElement.h"
#include "RenderTable.h"

namespace WebCore {

using namespace HTMLNames;

RenderTableCol::RenderTableCol(Node* node)
    : RenderBox(node)
    , m_span(1)
{
    // init RenderObject attributes
    setInline(true); // our object is not Inline
    updateFromElement();
}

void RenderTableCol::updateFromElement()
{
    int oldSpan = m_span;
    Node* n = node();
    if (n && (n->hasTagName(colTag) || n->hasTagName(colgroupTag))) {
        HTMLTableColElement* tc = static_cast<HTMLTableColElement*>(n);
        m_span = tc->span();
    } else
        m_span = !(style() && style()->display() == TABLE_COLUMN_GROUP);
    if (m_span != oldSpan && style() && parent())
        setNeedsLayoutAndPrefWidthsRecalc();
}

bool RenderTableCol::isChildAllowed(RenderObject* child, RenderStyle* style) const
{
    return !child->isText() && style && (style->display() == TABLE_COLUMN);
}

bool RenderTableCol::canHaveChildren() const
{
    // Cols cannot have children. This is actually necessary to fix a bug
    // with libraries.uc.edu, which makes a <p> be a table-column.
    return style()->display() == TABLE_COLUMN_GROUP;
}

IntRect RenderTableCol::clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer)
{
    // For now, just repaint the whole table.
    // FIXME: Find a better way to do this, e.g., need to repaint all the cells that we
    // might have propagated a background color or borders into.
    // FIXME: check for repaintContainer each time here?

    RenderTable* parentTable = table();
    if (!parentTable)
        return IntRect();
    return parentTable->clippedOverflowRectForRepaint(repaintContainer);
}

void RenderTableCol::imageChanged(WrappedImagePtr, const IntRect*)
{
    // FIXME: Repaint only the rect the image paints in.
    repaint();
}

void RenderTableCol::calcPrefWidths()
{
    setPrefWidthsDirty(false);

    for (RenderObject* child = firstChild(); child; child = child->nextSibling())
        child->setPrefWidthsDirty(false);
}

RenderTable* RenderTableCol::table() const
{
    RenderObject* table = parent();
    if (table && !table->isTable())
        table = table->parent();
    return table && table->isTable() ? toRenderTable(table) : 0;
}

}
