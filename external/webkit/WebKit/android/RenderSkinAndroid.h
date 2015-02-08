

#ifndef RenderSkinAndroid_h
#define RenderSkinAndroid_h

#include "PlatformString.h"

namespace android {
    class AssetManager;
}

class SkBitmap;

namespace WebCore {
class Node;
class PlatformGraphicsContext;

class RenderSkinAndroid
{
public:
    RenderSkinAndroid();
    virtual ~RenderSkinAndroid() {}
    
    enum State {
        kDisabled,
        kNormal,
        kFocused,
        kPressed,
    
        kNumStates
    };

    /**
     * Initialize the Android skinning system. The AssetManager may be used to find resources used
     * in rendering.
     */
    static void Init(android::AssetManager*, String drawableDirectory);
    
    /* DecodeBitmap determines which file to use, with the given fileName of the form 
     * "images/bitmap.png", and uses the asset manager to select the exact one.  It
     * returns true if it successfully decoded the bitmap, false otherwise.
     */
    static bool DecodeBitmap(android::AssetManager* am, const char* fileName, SkBitmap* bitmap);

    /*  draw() tells the skin to draw itself, and returns true if the skin needs
     *  a redraw to animations, false otherwise
     */
    virtual bool draw(PlatformGraphicsContext*) { return false; }
    
    /*  notifyState() checks to see if the element is checked, focused, and enabled
     *  it must be implemented in the subclass
     */
    virtual void notifyState(Node* element) { }
    
    /*  setDim() tells the skin its width and height
     */
    virtual void setDim(int width, int height) { m_width = width; m_height = height; }

protected:
    int                     m_height;
    int                     m_width;

};

} // WebCore

#endif
