

#ifndef RenderReplaced_h
#define RenderReplaced_h

#include "RenderBox.h"

namespace WebCore {

class RenderReplaced : public RenderBox {
public:
    RenderReplaced(Node*);
    RenderReplaced(Node*, const IntSize& intrinsicSize);
    virtual ~RenderReplaced();

protected:
    virtual void layout();

    virtual IntSize intrinsicSize() const;

    virtual void setSelectionState(SelectionState);

    bool isSelected() const;

    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    void setIntrinsicSize(const IntSize&);
    virtual void intrinsicSizeChanged();

    virtual void paint(PaintInfo&, int tx, int ty);
    bool shouldPaint(PaintInfo&, int& tx, int& ty);
    void adjustOverflowForBoxShadowAndReflect();
    IntRect localSelectionRect(bool checkWhetherSelected = true) const;

private:
    virtual const char* renderName() const { return "RenderReplaced"; }

    virtual bool canHaveChildren() const { return false; }

    virtual int lineHeight(bool firstLine, bool isRootLineBox = false) const;
    virtual int baselinePosition(bool firstLine, bool isRootLineBox = false) const;

    virtual void calcPrefWidths();

    virtual int minimumReplacedHeight() const { return 0; }

    virtual void paintReplaced(PaintInfo&, int /*tx*/, int /*ty*/) { }

    virtual IntRect clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer);

    virtual unsigned caretMaxRenderedOffset() const;
    virtual VisiblePosition positionForPoint(const IntPoint&);
    
    virtual bool canBeSelectionLeaf() const { return true; }

    virtual IntRect selectionRectForRepaint(RenderBoxModelObject* repaintContainer, bool clipToVisibleContent = true);

    IntSize m_intrinsicSize;
};

}

#endif
