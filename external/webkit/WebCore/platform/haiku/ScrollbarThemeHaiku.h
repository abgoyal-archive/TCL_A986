

#ifndef ScrollbarThemeHaiku_h
#define ScrollbarThemeHaiku_h

#include "ScrollbarThemeComposite.h"

namespace WebCore {
    class Scrollbar;

    class ScrollbarThemeHaiku : public ScrollbarThemeComposite {
        public:
            ScrollbarThemeHaiku();
            virtual ~ScrollbarThemeHaiku();

            virtual int scrollbarThickness(ScrollbarControlSize = RegularScrollbar);

            virtual bool hasButtons(Scrollbar*);
            virtual bool hasThumb(Scrollbar*);

            virtual IntRect backButtonRect(Scrollbar*, ScrollbarPart, bool painting);
            virtual IntRect forwardButtonRect(Scrollbar*, ScrollbarPart, bool painting);
            virtual IntRect trackRect(Scrollbar*, bool painting);

            virtual void paintScrollbarBackground(GraphicsContext*, Scrollbar*);
            virtual void paintButton(GraphicsContext*, Scrollbar*, const IntRect&, ScrollbarPart);
            virtual void paintThumb(GraphicsContext*, Scrollbar*, const IntRect&);
    };

}
#endif
