

#ifndef RenderSkinMediaButton_h
#define RenderSkinMediaButton_h

#include "RenderSkinAndroid.h"

class SkCanvas;

namespace WebCore {
class IntRect;
class RenderSkinMediaButton
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
    static void Draw(SkCanvas* , const IntRect& , int buttonType);
    /**
     * Button types
     */
    enum { PAUSE, PLAY, MUTE, REWIND, FORWARD, BACKGROUND_SLIDER, SLIDER_TRACK, SLIDER_THUMB };
    /**
     * Slider dimensions
     */
    static int sliderThumbWidth() { return 10; }
    static int sliderThumbHeight() { return 30; }

};

} // WebCore
#endif // RenderSkinMediaButton_h
