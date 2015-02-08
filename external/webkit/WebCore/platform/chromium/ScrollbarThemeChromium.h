

#ifndef ScrollbarThemeChromium_h
#define ScrollbarThemeChromium_h

#include "ScrollbarThemeComposite.h"

namespace WebCore {

    class PlatformMouseEvent;

    // This class contains the scrollbar code which is shared between Chromium
    // Windows and Linux.
    class ScrollbarThemeChromium : public ScrollbarThemeComposite {
    protected:
        virtual bool hasButtons(Scrollbar*) { return true; }
        virtual bool hasThumb(Scrollbar*);

        virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
        virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting = false);
        virtual IntRect trackRect(Scrollbar*, bool painting = false);

        virtual void paintScrollCorner(ScrollView*, GraphicsContext*, const IntRect&);

        virtual void paintTrackBackground(GraphicsContext*, Scrollbar*, const IntRect&);
        virtual void paintTickmarks(GraphicsContext*, Scrollbar*, const IntRect&);

        virtual IntSize buttonSize(Scrollbar*) = 0;
    };
} // namespace WebCore

#endif
