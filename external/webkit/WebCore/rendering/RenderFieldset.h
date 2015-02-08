

#ifndef RenderFieldset_h
#define RenderFieldset_h

#include "RenderBlock.h"

namespace WebCore {

class RenderFieldset : public RenderBlock {
public:
    RenderFieldset(Node*);

    RenderBox* findLegend() const;

private:
    virtual const char* renderName() const { return "RenderFieldSet"; }
    virtual bool isFieldset() const { return true; }

    virtual RenderObject* layoutLegend(bool relayoutChildren);

    virtual void calcPrefWidths();
    virtual bool avoidsFloats() const { return true; }
    virtual bool stretchesToMinIntrinsicWidth() const { return true; }

    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

    virtual void paintBoxDecorations(PaintInfo&, int tx, int ty);
    virtual void paintMask(PaintInfo&, int tx, int ty);

    void paintBorderMinusLegend(GraphicsContext*, int tx, int ty, int w, int h, const RenderStyle*, int lx, int lw, int lb);
};

inline RenderFieldset* toRenderFieldset(RenderObject* object)
{
    ASSERT(!object || object->isFieldset());
    return static_cast<RenderFieldset*>(object);
}

// This will catch anyone doing an unnecessary cast.
void toRenderFieldset(const RenderFieldset*);

} // namespace WebCore

#endif // RenderFieldset_h
