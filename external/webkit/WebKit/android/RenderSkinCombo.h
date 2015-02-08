

#ifndef RenderSkinCombo_h
#define RenderSkinCombo_h

#include "RenderSkinAndroid.h"
#include "SkRect.h"

class SkCanvas;

namespace WebCore {

// This is very similar to RenderSkinButton - maybe they should be the same class?
class RenderSkinCombo : public RenderSkinAndroid
{
public:
    /**
     * Initialize the class before use. Uses the AssetManager to initialize any bitmaps the class may use.
     */
    static void Init(android::AssetManager*, String drawableDirectory);

    /**
     * Draw the provided Node on the SkCanvas, using the dimensions provided by
     * x,y,w,h.  Return true if we did not draw, and WebKit needs to draw it,
     * false otherwise.
     */
    static bool Draw(SkCanvas* , Node* , int x, int y, int w, int h);

    // The image is wider than the RenderObject, so this accounts for that.
    static int extraWidth() { return arrowMargin[resolution]; }
    static int padding() { return padMargin[resolution]; }

    enum Resolution {
        MedRes,
        HighRes
    };
private:
    static Resolution resolution;
    const static int arrowMargin[2];
    const static int padMargin[2];
    const static SkIRect margin[2][2];
    const static SkIRect margin_small[2][2];
}; 

} // WebCore

#endif
