

#ifndef RenderSkinRadio_h
#define RenderSkinRadio_h

#include "PlatformString.h"

class SkCanvas;

namespace android {
    class AssetManager;
}

namespace WebCore {

class Node;
class IntRect;

class RenderSkinRadio
{
public:
    /**
     * Initialize the class before use. Uses the AssetManager to initialize any bitmaps the class may use.
     */
    static void Init(android::AssetManager*, String drawableDirectory);

    /**
     * Draw the element to the canvas at the specified size and location.
     * param isCheckBox If true, draws a checkbox.  Else, draw a radio button.
     */
    static void Draw(SkCanvas* canvas, Node* element, const IntRect&,
            bool isCheckBox);
};

} // WebCore
#endif
