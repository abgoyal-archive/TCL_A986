

#ifndef ChromeClientChromium_h
#define ChromeClientChromium_h

#include "ChromeClient.h"
#include <wtf/Forward.h>

namespace WebCore {
class AccessibilityObject;
class IntRect;
class PopupContainer;

// Contains Chromium-specific extensions to the ChromeClient.  Only put
// things here that don't make sense for other ports.
class ChromeClientChromium : public ChromeClient {
public:
    // Notifies the client of a new popup widget.  The client should place
    // and size the widget with the given bounds, relative to the screen.
    // If handleExternal is true, then drawing and input handling for the
    // popup will be handled by the external embedder.
    virtual void popupOpened(PopupContainer* popupContainer, const IntRect& bounds,
                             bool focusOnShow, bool handleExternal) = 0;

    // Notifies embedder that the state of an accessibility object has changed.
    virtual void didChangeAccessibilityObjectState(AccessibilityObject*) = 0;
};

} // namespace WebCore

#endif
