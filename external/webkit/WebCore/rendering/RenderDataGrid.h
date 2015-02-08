

#ifndef RenderDataGrid_h
#define RenderDataGrid_h

#if ENABLE(DATAGRID)

#include "HTMLDataGridElement.h"
#include "RenderBlock.h"
#include "ScrollbarClient.h"
#include "StyleImage.h"
#include <wtf/RefPtr.h>
#include <wtf/Vector.h>

namespace WebCore {

class RenderDataGrid : public RenderBlock, private ScrollbarClient {
public:
    RenderDataGrid(Element*);
    ~RenderDataGrid();
    
    virtual const char* renderName() const { return "RenderDataGrid"; }
    virtual bool canHaveChildren() const { return false; }
    virtual void calcPrefWidths();
    virtual void layout();
    virtual void paintObject(PaintInfo&, int tx, int ty);

    void columnsChanged();

private:
    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    RenderStyle* columnStyle(DataGridColumn*);
    RenderStyle* headerStyle(DataGridColumn*);
    void recalcStyleForColumns();
    void recalcStyleForColumn(DataGridColumn*);

    void layoutColumns();
    void paintColumnHeaders(PaintInfo&, int tx, int ty);
    void paintColumnHeader(DataGridColumn*, PaintInfo&, int tx, int ty);

    HTMLDataGridElement* gridElement() const { return static_cast<HTMLDataGridElement*>(node()); }

    // ScrollbarClient interface.
    virtual void valueChanged(Scrollbar*);
    virtual void invalidateScrollbarRect(Scrollbar*, const IntRect&);
    virtual bool isActive() const;
    virtual bool scrollbarCornerPresent() const { return false; } // We don't support resize on data grids yet.  If we did this would have to change.
    virtual IntRect convertFromScrollbarToContainingView(const Scrollbar*, const IntRect&) const;
    virtual IntRect convertFromContainingViewToScrollbar(const Scrollbar*, const IntRect&) const;
    virtual IntPoint convertFromScrollbarToContainingView(const Scrollbar*, const IntPoint&) const;
    virtual IntPoint convertFromContainingViewToScrollbar(const Scrollbar*, const IntPoint&) const;

    RefPtr<Scrollbar> m_vBar;
};

}

#endif

#endif // RenderDataGrid_h
