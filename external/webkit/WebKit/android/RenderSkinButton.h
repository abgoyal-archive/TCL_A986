

#ifndef RenderSkinButton_h
#define RenderSkinButton_h

#include "RenderSkinAndroid.h"

class SkCanvas;

namespace WebCore {
class IntRect;
class RenderSkinButton
{
public:
    /**
     * Initialize the class before use. Uses the AssetManager to initialize any 
     * bitmaps the class may use.
     */
    static void Init(android::AssetManager*, String drawableDirectory);
    /**
     * Draw the skin to the canvas, using the rectangle for its bounds and the 
     * State to determine which skin to use, i.e. focused or not focused.
     */
    static void Draw(SkCanvas* , const IntRect& , RenderSkinAndroid::State);
};

} // WebCore
#endif
