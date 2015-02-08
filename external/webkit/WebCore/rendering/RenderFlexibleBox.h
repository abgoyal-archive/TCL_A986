

#ifndef RenderFlexibleBox_h
#define RenderFlexibleBox_h

#include "RenderBlock.h"

namespace WebCore {

class RenderFlexibleBox : public RenderBlock {
public:
    RenderFlexibleBox(Node*);
    virtual ~RenderFlexibleBox();

    virtual const char* renderName() const;

    virtual void calcPrefWidths();
    void calcHorizontalPrefWidths();
    void calcVerticalPrefWidths();

    virtual void layoutBlock(bool relayoutChildren);
    void layoutHorizontalBox(bool relayoutChildren);
    void layoutVerticalBox(bool relayoutChildren);

    virtual bool avoidsFloats() const { return true; }

    virtual bool isFlexibleBox() const { return true; }
    virtual bool isFlexingChildren() const { return m_flexingChildren; }
    virtual bool isStretchingChildren() const { return m_stretchingChildren; }

    void placeChild(RenderBox* child, int x, int y);

protected:
    int allowedChildFlex(RenderBox* child, bool expanding, unsigned group);

    bool hasMultipleLines() const { return style()->boxLines() == MULTIPLE; }
    bool isVertical() const { return style()->boxOrient() == VERTICAL; }
    bool isHorizontal() const { return style()->boxOrient() == HORIZONTAL; }

    bool m_flexingChildren : 1;
    bool m_stretchingChildren : 1;
};

} // namespace WebCore

#endif // RenderFlexibleBox_h
