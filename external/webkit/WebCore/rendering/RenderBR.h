

#ifndef RenderBR_h
#define RenderBR_h

#include "RenderText.h"

namespace WebCore {

class Position;

class RenderBR : public RenderText {
public:
    RenderBR(Node*);
    virtual ~RenderBR();

    virtual const char* renderName() const { return "RenderBR"; }
 
    virtual IntRect selectionRectForRepaint(RenderBoxModelObject* /*repaintContainer*/, bool /*clipToVisibleContent*/) { return IntRect(); }

    virtual unsigned width(unsigned /*from*/, unsigned /*len*/, const Font&, int /*xpos*/) const { return 0; }
    virtual unsigned width(unsigned /*from*/, unsigned /*len*/, int /*xpos*/, bool /*firstLine = false*/) const { return 0; }

    virtual int lineHeight(bool firstLine, bool isRootLineBox = false) const;
    virtual int baselinePosition(bool firstLine, bool isRootLineBox = false) const;

    // overrides
    virtual bool isBR() const { return true; }

    virtual int caretMinOffset() const;
    virtual int caretMaxOffset() const;
    virtual unsigned caretMaxRenderedOffset() const;

    virtual VisiblePosition positionForPoint(const IntPoint&);

protected:
    virtual void styleDidChange(StyleDifference, const RenderStyle* oldStyle);

private:
    mutable int m_lineHeight;
};

} // namespace WebCore

#endif // RenderBR_h
