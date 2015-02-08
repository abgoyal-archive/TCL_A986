

#ifndef SVGInlineFlowBox_h
#define SVGInlineFlowBox_h

#if ENABLE(SVG)
#include "InlineFlowBox.h"

namespace WebCore {

class SVGInlineFlowBox : public InlineFlowBox {
public:
    SVGInlineFlowBox(RenderObject* obj)
        : InlineFlowBox(obj)
        , m_height(0)
    {
    }

    virtual int virtualHeight() const { return m_height; }
    void setHeight(int h) { m_height = h; }

    virtual void paint(RenderObject::PaintInfo&, int tx, int ty);
    virtual int placeBoxesHorizontally(int x, int& leftPosition, int& rightPosition, bool& needsWordSpacing);
    virtual int verticallyAlignBoxes(int heightOfBlock);
    
private:
    int m_height;
};

} // namespace WebCore

#endif // ENABLE(SVG)

#endif // SVGInlineFlowBox_h
