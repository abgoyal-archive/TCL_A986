

#include "config.h"

#if ENABLE(SVG)
#include "SVGInlineFlowBox.h"
#include "SVGNames.h"

namespace WebCore {

using namespace SVGNames;

void SVGInlineFlowBox::paint(RenderObject::PaintInfo&, int, int)
{
    ASSERT_NOT_REACHED();
}

int SVGInlineFlowBox::placeBoxesHorizontally(int, int&, int&, bool&)
{
    // no-op
    return 0;
}

int SVGInlineFlowBox::verticallyAlignBoxes(int)
{
    // no-op
    return 0;
}

} // namespace WebCore

#endif // ENABLE(SVG)
