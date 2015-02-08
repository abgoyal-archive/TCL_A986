

#ifndef RenderThemeHaiku_h
#define RenderThemeHaiku_h

#include "RenderTheme.h"

namespace WebCore {

    class RenderThemeHaiku : public RenderTheme {
    private:
        RenderThemeHaiku();
        virtual ~RenderThemeHaiku();

    public:
        static PassRefPtr<RenderTheme> create();

        // A method asking if the theme's controls actually care about redrawing when hovered.
        virtual bool supportsHover(const RenderStyle* style) const { return false; }

        // A method asking if the theme is able to draw the focus ring.
        virtual bool supportsFocusRing(const RenderStyle*) const;

        // The platform selection color.
        virtual Color platformActiveSelectionBackgroundColor() const;
        virtual Color platformInactiveSelectionBackgroundColor() const;
        virtual Color platformActiveSelectionForegroundColor() const;
        virtual Color platformInactiveSelectionForegroundColor() const;

        virtual Color platformTextSearchHighlightColor() const;

        // System fonts.
        virtual void systemFont(int propId, FontDescription&) const;

    protected:
        virtual bool paintCheckbox(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual void setCheckboxSize(RenderStyle*) const;

        virtual bool paintRadio(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual void setRadioSize(RenderStyle*) const;

        virtual void adjustMenuListStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
        virtual bool paintMenuList(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    };

} // namespace WebCore

#endif // RenderThemeHaiku_h
