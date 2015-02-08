

#ifndef WebThemeEngine_h
#define WebThemeEngine_h

#include "../WebCanvas.h"
#include "../WebColor.h"

namespace WebKit {

struct WebRect;

// The part and state parameters correspond to values defined by the
// Windows Theme API (see
// http://msdn.microsoft.com/en-us/library/bb773187(VS.85).aspx ).
// The classicState parameter corresponds to the uState
// parameter of the Windows DrawFrameControl() function.
// See the definitions in <vsstyle.h> and <winuser.h>.
class WebThemeEngine {
public:
    virtual void paintButton(
        WebCanvas*, int part, int state, int classicState,
        const WebRect&) = 0;

    virtual void paintMenuList(
        WebCanvas*, int part, int state, int classicState,
        const WebRect&) = 0;

    virtual void paintScrollbarArrow(
        WebCanvas*, int state, int classicState,
        const WebRect&) = 0;

    virtual void paintScrollbarThumb(
        WebCanvas*, int part, int state, int classicState,
        const WebRect&) = 0;

    virtual void paintScrollbarTrack(
        WebCanvas*, int part, int state, int classicState,
        const WebRect&, const WebRect& alignRect) = 0;

    virtual void paintTextField(
        WebCanvas*, int part, int state, int classicState,
        const WebRect&, WebColor, bool fillContentArea, bool drawEdges) = 0;

    virtual void paintTrackbar(
        WebCanvas*, int part, int state, int classicState,
        const WebRect&) = 0;
};

} // namespace WebKit

#endif
