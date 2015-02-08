

#ifndef RenderTableCell_h
#define RenderTableCell_h

#include "RenderTableSection.h"

namespace WebCore {

class RenderTableCell : public RenderBlock {
public:
    RenderTableCell(Node*);

    // FIXME: need to implement cellIndex
    int cellIndex() const { return 0; }
    void setCellIndex(int) { }

    int colSpan() const { return m_columnSpan; }
    void setColSpan(int c) { m_columnSpan = c; }

    int rowSpan() const { return m_rowSpan; }
    void setRowSpan(int r) { m_rowSpan = r; }

    int col() const { return m_column; }
    void setCol(int col) { m_column = col; }
    int row() const { return m_row; }
    void setRow(int row) { m_row = row; }

    RenderTableSection* section() const { return toRenderTableSection(parent()->parent()); }
    RenderTable* table() const { return toRenderTable(parent()->parent()->parent()); }

    Length styleOrColWidth() const;

    virtual void calcPrefWidths();

#if PLATFORM(ANDROID)
#ifdef ANDROID_LAYOUT
    // RenderTableSection needs to access this in setCellWidths()
    int getVisibleWidth() { return m_visibleWidth; }
#endif
#endif

    void updateWidth(int);

    int borderLeft() const;
    int borderRight() const;
    int borderTop() const;
    int borderBottom() const;

    int borderHalfLeft(bool outer) const;
    int borderHalfRight(bool outer) const;
    int borderHalfTop(bool outer) const;
    int borderHalfBottom(bool outer) const;

    CollapsedBorderValue collapsedLeftBorder(bool rtl) const;
    CollapsedBorderValue collapsedRightBorder(bool rtl) const;
    CollapsedBorderValue collapsedTopBorder() const;
    CollapsedBorderValue collapsedBottomBorder() const;

    typedef Vector<CollapsedBorderValue, 100> CollapsedBorderStyles;
    void collectBorderStyles(CollapsedBorderStyles&) const;
    static void sortBorderStyles(CollapsedBorderStyles&);

    virtual void updateFromElement();

    virtual void layout();

    virtual void paint(PaintInfo&, int tx, int ty);

    void paintBackgroundsBehindCell(PaintInfo&, int tx, int ty, RenderObject* backgroundObject);

    virtual int baselinePosition(bool firstLine = false, bool isRootLineBox = false) const;

    void setIntrinsicPaddingTop(int p) { m_intrinsicPaddingTop = p; }
    void setIntrinsicPaddingBottom(int p) { m_intrinsicPaddingBottom = p; }
    void setIntrinsicPadding(int top, int bottom) { setIntrinsicPaddingTop(top); setIntrinsicPaddingBottom(bottom); }
    void clearIntrinsicPadding() { setIntrinsicPadding(0, 0); }

    int intrinsicPaddingTop() const { return m_intrinsicPaddingTop; }
    int intrinsicPaddingBottom() const { return m_intrinsicPaddingBottom; }

    virtual int paddingTop(bool includeIntrinsicPadding = true) const;
    virtual int paddingBottom(bool includeIntrinsicPadding = true) const;

    virtual void setOverrideSize(int);

    bool hasVisibleOverflow() const { return m_overflow; }

protected:
    virtual void styleWillChange(StyleDifference, const RenderStyle* newStyle);
    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

private:
    virtual const char* renderName() const { return isAnonymous() ? "RenderTableCell (anonymous)" : "RenderTableCell"; }

    virtual bool isTableCell() const { return true; }

    virtual void destroy();

    virtual bool requiresLayer() const { return isPositioned() || isTransparent() || hasOverflowClip() || hasTransform() || hasMask() || hasReflection(); }

    virtual void calcWidth();

    virtual void paintBoxDecorations(PaintInfo&, int tx, int ty);
    virtual void paintMask(PaintInfo&, int tx, int ty);

    virtual IntSize offsetFromContainer(RenderObject*) const;
    virtual IntRect clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer);
    virtual void computeRectForRepaint(RenderBoxModelObject* repaintContainer, IntRect&, bool fixed = false);

    void paintCollapsedBorder(GraphicsContext*, int x, int y, int w, int h);

    int m_row;
    int m_column;
    int m_rowSpan;
    int m_columnSpan;
    int m_intrinsicPaddingTop;
    int m_intrinsicPaddingBottom;
    int m_percentageHeight;
};

inline RenderTableCell* toRenderTableCell(RenderObject* object)
{
    ASSERT(!object || object->isTableCell());
    return static_cast<RenderTableCell*>(object);
}

inline const RenderTableCell* toRenderTableCell(const RenderObject* object)
{
    ASSERT(!object || object->isTableCell());
    return static_cast<const RenderTableCell*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderTableCell(const RenderTableCell*);

} // namespace WebCore

#endif // RenderTableCell_h
