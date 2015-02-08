

#ifndef RenderThemeAndroid_h
#define RenderThemeAndroid_h

#include "RenderTheme.h"

namespace WebCore {

class RenderSkinButton;
class RenderSkinRadio;
class RenderSkinCombo;

struct ThemeData {
    ThemeData()
        : m_part(0)
        , m_state(0)
    {
    }

    unsigned m_part;
    unsigned m_state;
};

class RenderThemeAndroid : public RenderTheme {
public:
    static PassRefPtr<RenderTheme> create();
    ~RenderThemeAndroid();
    
    virtual bool stateChanged(RenderObject*, ControlState) const;
       
    virtual bool supportsFocusRing(const RenderStyle*) const;
    // A method asking if the theme's controls actually care about redrawing when hovered.
    virtual bool supportsHover(const RenderStyle* style) const { return style->affectedByHoverRules(); }

    virtual int baselinePosition(const RenderObject*) const;

    virtual Color platformActiveSelectionBackgroundColor() const;
    virtual Color platformInactiveSelectionBackgroundColor() const;
    virtual Color platformActiveSelectionForegroundColor() const;
    virtual Color platformInactiveSelectionForegroundColor() const;
    virtual Color platformTextSearchHighlightColor() const;

    virtual Color platformActiveListBoxSelectionBackgroundColor() const;
    virtual Color platformInactiveListBoxSelectionBackgroundColor() const;
    virtual Color platformActiveListBoxSelectionForegroundColor() const;
    virtual Color platformInactiveListBoxSelectionForegroundColor() const;

    virtual void systemFont(int, WebCore::FontDescription&) const {}

    virtual int minimumMenuListSize(RenderStyle*) const { return 0; }

protected:
    virtual bool paintCheckbox(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual void setCheckboxSize(RenderStyle*) const;

#if ENABLE(VIDEO)
    virtual String extraMediaControlsStyleSheet();
    virtual void adjustSliderThumbSize(RenderObject* o) const;
    virtual bool shouldRenderMediaControlPart(ControlPart part, Element* e);
    bool paintMediaMuteButton(RenderObject* o, const RenderObject::PaintInfo& paintInfo, const IntRect& r);
    bool paintMediaPlayButton(RenderObject* o, const RenderObject::PaintInfo& paintInfo, const IntRect& r);
    bool paintMediaSeekBackButton(RenderObject* o, const RenderObject::PaintInfo& paintInfo, const IntRect& r);
    bool paintMediaSeekForwardButton(RenderObject* o, const RenderObject::PaintInfo& paintInfo, const IntRect& r);
    bool paintMediaSliderTrack(RenderObject* o, const RenderObject::PaintInfo& paintInfo, const IntRect& r);
    bool paintMediaSliderThumb(RenderObject* o, const RenderObject::PaintInfo& paintInfo, const IntRect& r);
    virtual bool paintMediaControlsBackground(RenderObject* object, const RenderObject::PaintInfo& paintInfo, const IntRect& rect);
#endif

    virtual bool paintRadio(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual void setRadioSize(RenderStyle*) const;

    virtual void adjustButtonStyle(CSSStyleSelector*, RenderStyle*, WebCore::Element*) const;
    virtual bool paintButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustTextFieldStyle(CSSStyleSelector*, RenderStyle*, WebCore::Element*) const;
    virtual bool paintTextField(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustTextAreaStyle(CSSStyleSelector*, RenderStyle*, WebCore::Element*) const;
    virtual bool paintTextArea(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    
    bool paintCombo(RenderObject*, const RenderObject::PaintInfo&,  const IntRect&);

    virtual void adjustListboxStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual void adjustMenuListStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintMenuList(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustMenuListButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintMenuListButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    
    virtual void adjustSearchFieldStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintSearchField(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

private:
    RenderThemeAndroid();
    void addIntrinsicMargins(RenderStyle*) const;
    void close();

    bool supportsFocus(ControlPart);
    friend RenderTheme* theme();
};

} // namespace WebCore

#endif // RenderThemeAndroid_h
