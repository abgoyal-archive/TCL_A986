

#ifndef RenderThemeChromiumSkia_h
#define RenderThemeChromiumSkia_h

#include "RenderTheme.h"

namespace WebCore {

    class RenderThemeChromiumSkia : public RenderTheme {
    public:
        RenderThemeChromiumSkia();
        virtual ~RenderThemeChromiumSkia();

        virtual String extraDefaultStyleSheet();
        virtual String extraQuirksStyleSheet();
#if ENABLE(VIDEO)
        virtual String extraMediaControlsStyleSheet();
#endif

        // A method asking if the theme's controls actually care about redrawing when hovered.
        virtual bool supportsHover(const RenderStyle*) const;

        // A method asking if the theme is able to draw the focus ring.
        virtual bool supportsFocusRing(const RenderStyle*) const;

        // The platform selection color.
        virtual Color platformActiveSelectionBackgroundColor() const;
        virtual Color platformInactiveSelectionBackgroundColor() const;
        virtual Color platformActiveSelectionForegroundColor() const;
        virtual Color platformInactiveSelectionForegroundColor() const;
        virtual Color platformFocusRingColor() const;

        // To change the blink interval, override caretBlinkIntervalInternal instead of this one so that we may share layout test code an intercepts.
        virtual double caretBlinkInterval() const;

        // System fonts.
        virtual void systemFont(int propId, FontDescription&) const;

        virtual int minimumMenuListSize(RenderStyle*) const;

        virtual bool paintCheckbox(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual void setCheckboxSize(RenderStyle*) const;

        virtual bool paintRadio(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual void setRadioSize(RenderStyle*) const;

        virtual bool paintButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual void adjustButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;

        virtual bool paintTextField(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        virtual bool paintTextArea(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        virtual void adjustSearchFieldStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
        virtual bool paintSearchField(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        virtual void adjustSearchFieldCancelButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
        virtual bool paintSearchFieldCancelButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        virtual void adjustSearchFieldDecorationStyle(CSSStyleSelector*, RenderStyle*, Element*) const;

        virtual void adjustSearchFieldResultsDecorationStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
        virtual bool paintSearchFieldResultsDecoration(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        virtual void adjustSearchFieldResultsButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
        virtual bool paintSearchFieldResultsButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        virtual bool paintMediaControlsBackground(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual bool paintMediaSliderTrack(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual bool paintMediaVolumeSliderTrack(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual void adjustSliderThumbSize(RenderObject*) const;
        virtual bool paintMediaSliderThumb(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual bool paintMediaVolumeSliderThumb(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual bool paintMediaPlayButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual bool paintMediaMuteButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        // MenuList refers to an unstyled menulist (meaning a menulist without
        // background-color or border set) and MenuListButton refers to a styled
        // menulist (a menulist with background-color or border set). They have
        // this distinction to support showing aqua style themes whenever they
        // possibly can, which is something we don't want to replicate.
        //
        // In short, we either go down the MenuList code path or the MenuListButton
        // codepath. We never go down both. And in both cases, they render the
        // entire menulist.
        virtual void adjustMenuListStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
        virtual bool paintMenuList(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual void adjustMenuListButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
        virtual bool paintMenuListButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        virtual bool paintSliderTrack(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
        virtual bool paintSliderThumb(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

        // These methods define the padding for the MenuList's inner block.
        virtual int popupInternalPaddingLeft(RenderStyle*) const;
        virtual int popupInternalPaddingRight(RenderStyle*) const;
        virtual int popupInternalPaddingTop(RenderStyle*) const;
        virtual int popupInternalPaddingBottom(RenderStyle*) const;

        virtual int buttonInternalPaddingLeft() const;
        virtual int buttonInternalPaddingRight() const;
        virtual int buttonInternalPaddingTop() const;
        virtual int buttonInternalPaddingBottom() const;

#if ENABLE(VIDEO)
        // Media controls
        virtual bool shouldRenderMediaControlPart(ControlPart, Element*);
#endif

        // Provide a way to pass the default font size from the Settings object
        // to the render theme.  FIXME: http://b/1129186 A cleaner way would be
        // to remove the default font size from this object and have callers
        // that need the value to get it directly from the appropriate Settings
        // object.
        static void setDefaultFontSize(int);

    protected:
        static const String& defaultGUIFont();

        // The default variable-width font size.  We use this as the default font
        // size for the "system font", and as a base size (which we then shrink) for
        // form control fonts.
        static float defaultFontSize;

        virtual double caretBlinkIntervalInternal() const;

    private:
        int menuListInternalPadding(RenderStyle*, int paddingType) const;
        bool paintMediaButtonInternal(GraphicsContext*, const IntRect&, Image*);
    };

} // namespace WebCore

#endif // RenderThemeChromiumSkia_h
