

#ifndef RenderTableCol_h
#define RenderTableCol_h

#include "RenderBox.h"

namespace WebCore {

class RenderTable;

class RenderTableCol : public RenderBox {
public:
    RenderTableCol(Node*);

    const RenderObjectChildList* children() const { return &m_children; }
    RenderObjectChildList* children() { return &m_children; }

    virtual void calcPrefWidths();

    int span() const { return m_span; }
    void setSpan(int span) { m_span = span; }

private:
    virtual RenderObjectChildList* virtualChildren() { return children(); }
    virtual const RenderObjectChildList* virtualChildren() const { return children(); }

    virtual const char* renderName() const { return "RenderTableCol"; }
    virtual bool isTableCol() const { return true; }
    virtual int lineHeight(bool) const { return 0; }
    virtual void updateFromElement();

    virtual bool isChildAllowed(RenderObject*, RenderStyle*) const;
    virtual bool canHaveChildren() const;
    virtual bool requiresLayer() const { return false; }

    virtual IntRect clippedOverflowRectForRepaint(RenderBoxModelObject* repaintContainer);
    virtual void imageChanged(WrappedImagePtr, const IntRect* = 0);

    RenderTable* table() const;

    RenderObjectChildList m_children;
    int m_span;
};

inline RenderTableCol* toRenderTableCol(RenderObject* object)
{
    ASSERT(!object || object->isTableCol());
    return static_cast<RenderTableCol*>(object);
}

inline const RenderTableCol* toRenderTableCol(const RenderObject* object)
{
    ASSERT(!object || object->isTableCol());
    return static_cast<const RenderTableCol*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderTableCol(const RenderTableCol*);

}

#endif
