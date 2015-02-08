

#include "config.h"
#include "FrameWin.h"

#include "Bridge.h"
#include "Document.h"
#include "FloatRect.h"
#include "PrintContext.h"
#include "RenderView.h"
#include "Settings.h"
#include "TransformationMatrix.h"

namespace WebCore {

void computePageRectsForFrame(Frame* frame, const IntRect& printRect, float headerHeight, float footerHeight, float userScaleFactor, Vector<IntRect>& outPages, int& outPageHeight)
{
    PrintContext printContext(frame);
    float pageHeight = 0;
    printContext.computePageRects(printRect, headerHeight, footerHeight, userScaleFactor, pageHeight);
    outPageHeight = static_cast<int>(pageHeight);
    outPages = printContext.pageRects();
}

DragImageRef Frame::dragImageForSelection()
{    
    if (selection()->isRange())
        return imageFromSelection(this, false);

    return 0;
}

} // namespace WebCore
