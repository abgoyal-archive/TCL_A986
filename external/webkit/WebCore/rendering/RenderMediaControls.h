

#ifndef RenderMediaControls_h
#define RenderMediaControls_h

#include "RenderObject.h"
#include "MediaControlElements.h"

namespace WebCore {

class HTMLMediaElement;
class RenderMediaControls {
public:
    static bool paintMediaControlsPart(MediaControlElementType, RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    static void adjustMediaSliderThumbSize(RenderObject*);
};

} // namespace WebCore

#endif // RenderMediaControls_h
