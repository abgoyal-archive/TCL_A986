

#ifndef FrameWin_H
#define FrameWin_H

#include <wtf/Vector.h>

// Forward declared so we don't need wingdi.h.
typedef struct HBITMAP__* HBITMAP;

namespace WebCore {

    class Frame;
    class IntRect;

    HBITMAP imageFromSelection(Frame* frame, bool forceWhiteText);
    void computePageRectsForFrame(Frame*, const IntRect& printRect, float headerHeight, float footerHeight, float userScaleFactor, Vector<IntRect>& outPages, int& outPageHeight);

}

#endif
