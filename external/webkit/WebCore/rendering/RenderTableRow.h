

#ifndef RenderTableRow_h
#define RenderTableRow_h

#include "RenderTableSection.h"

namespace WebCore {

class RenderTableRow : public RenderBox {
public:
    RenderTableRow(Node*);

    const RenderObjectChildList* children() const { return &m_children; }
    RenderObjectChildList* children() { return &m_children; }

    RenderTableSection* section() const { return toRenderTableSection(parent()); }
    RenderTable* table() const { return toRenderTable(parent()->parent()); }

private:
    virtual RenderObjectChildList* virtualChildren() { return children(); }
    virtual const RenderObjectChildList* virtualChildren() const { return children(); }

    virtual const char* renderName() const { return isAnonymous() ? "RenderTableRow (anonymous)" : "RenderTableRow"; }

    virtual bool isTableRow() const { return true; }

    virtual void destroy();

    virtual void addChild(RenderObject* child, RenderObject* beforeChild = 0);
    virtual int lineHeight(bool, bool) const { return 0; }
    virtual void layout();
    virtual IntRect clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer);
    virtual bool nodeAtPoint(const HitTestRequest&, HitTestResult&, int x, int y, int tx, int ty, HitTestAction);

    // The only time rows get a layer is when they have transparency.
    virtual bool requiresLayer() const { return isTransparent() || hasOverflowClip() || hasTransform() || hasMask(); }

    virtual void paint(PaintInfo&, int tx, int ty);

    virtual void imageChanged(WrappedImagePtr, const IntRect* = 0);

    virtual void styleWillChange(StyleDifference, const RenderStyle* newStyle);

    RenderObjectChildList m_children;
};

inline RenderTableRow* toRenderTableRow(RenderObject* object)
{
    ASSERT(!object || object->isTableRow());
    return static_cast<RenderTableRow*>(object);
}

inline const RenderTableRow* toRenderTableRow(const RenderObject* object)
{
    ASSERT(!object || object->isTableRow());
    return static_cast<const RenderTableRow*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderTableRow(const RenderTableRow*);

} // namespace WebCore

#endif // RenderTableRow_h
