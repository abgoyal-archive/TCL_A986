

#ifndef RenderMediaControlsChromium_h
#define RenderMediaControlsChromium_h

#include "RenderObject.h"
#include "MediaControlElements.h"

namespace WebCore {

class HTMLMediaElement;
class RenderMediaControlsChromium {
public:
    static bool shouldRenderMediaControlPart(ControlPart, Element*);
    static bool paintMediaControlsPart(MediaControlElementType, RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    static void adjustMediaSliderThumbSize(RenderObject*);
};

} // namespace WebCore

#endif // RenderMediaControlsChromium_h
