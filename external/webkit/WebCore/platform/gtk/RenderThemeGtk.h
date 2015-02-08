

#ifndef RenderThemeGtk_h
#define RenderThemeGtk_h

#include "GRefPtr.h"
#include "RenderTheme.h"

typedef struct _GtkWidget GtkWidget;
typedef struct _GtkStyle GtkStyle;
typedef struct _GtkContainer GtkContainer;
typedef struct _GdkRectangle GdkRectangle;
typedef struct _GdkDrawable GdkDrawable;
typedef struct _GtkBorder GtkBorder;
typedef struct _GtkThemeParts GtkThemeParts;

namespace WebCore {

class RenderThemeGtk : public RenderTheme {
private:
    RenderThemeGtk();
    virtual ~RenderThemeGtk();

public:
    static PassRefPtr<RenderTheme> create();

    // A method asking if the theme's controls actually care about redrawing when hovered.
    virtual bool supportsHover(const RenderStyle* style) const { return true; }

    // A method asking if the theme is able to draw the focus ring.
    virtual bool supportsFocusRing(const RenderStyle*) const;

    // A method asking if the control changes its tint when the window has focus or not.
    virtual bool controlSupportsTints(const RenderObject*) const;

    // A general method asking if any control tinting is supported at all.
    virtual bool supportsControlTints() const { return true; }

    // A method to obtain the baseline position for a "leaf" control.  This will only be used if a baseline
    // position cannot be determined by examining child content. Checkboxes and radio buttons are examples of
    // controls that need to do this.
    virtual int baselinePosition(const RenderObject*) const;

    // The platform selection color.
    virtual Color platformActiveSelectionBackgroundColor() const;
    virtual Color platformInactiveSelectionBackgroundColor() const;
    virtual Color platformActiveSelectionForegroundColor() const;
    virtual Color platformInactiveSelectionForegroundColor() const;

    // List Box selection color
    virtual Color activeListBoxSelectionBackgroundColor() const;
    virtual Color activeListBoxSelectionForegroundColor() const;
    virtual Color inactiveListBoxSelectionBackgroundColor() const;
    virtual Color inactiveListBoxSelectionForegroundColor() const;

    virtual double caretBlinkInterval() const;

    virtual void platformColorsDidChange();

    // System fonts.
    virtual void systemFont(int propId, FontDescription&) const;

#if ENABLE(VIDEO)
    virtual String extraMediaControlsStyleSheet();
#endif

    GtkThemeParts* partsForDrawable(GdkDrawable*) const;

protected:
    virtual bool paintCheckbox(RenderObject* o, const RenderObject::PaintInfo& i, const IntRect& r);
    virtual void setCheckboxSize(RenderStyle* style) const;

    virtual bool paintRadio(RenderObject* o, const RenderObject::PaintInfo& i, const IntRect& r);
    virtual void setRadioSize(RenderStyle* style) const;

    virtual void adjustButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustTextFieldStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintTextField(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual bool paintTextArea(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustMenuListStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintMenuList(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustSearchFieldResultsDecorationStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintSearchFieldResultsDecoration(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustSearchFieldStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintSearchField(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustSearchFieldResultsButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintSearchFieldResultsButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustSearchFieldCancelButtonStyle(CSSStyleSelector*, RenderStyle*, Element*) const;
    virtual bool paintSearchFieldCancelButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);

    virtual void adjustSliderThumbSize(RenderObject*) const;

#if ENABLE(VIDEO)
    virtual void initMediaStyling(GtkStyle* style, bool force);
    virtual bool paintMediaFullscreenButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual bool paintMediaPlayButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual bool paintMediaMuteButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual bool paintMediaSeekBackButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual bool paintMediaSeekForwardButton(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual bool paintMediaSliderTrack(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
    virtual bool paintMediaSliderThumb(RenderObject*, const RenderObject::PaintInfo&, const IntRect&);
#endif

private:
    /*
     * hold the state
     */
    GtkWidget* gtkEntry() const;
    GtkWidget* gtkTreeView() const;

    /*
     * unmapped GdkWindow having a container. This is holding all
     * our fake widgets
     */
    GtkContainer* gtkContainer() const;

    mutable GtkWidget* m_gtkWindow;
    mutable GtkContainer* m_gtkContainer;
    mutable GtkWidget* m_gtkEntry;
    mutable GtkWidget* m_gtkTreeView;

    mutable Color m_panelColor;
    mutable Color m_sliderColor;
    mutable Color m_sliderThumbColor;

    const int m_mediaIconSize;
    const int m_mediaSliderHeight;
    const int m_mediaSliderThumbWidth;
    const int m_mediaSliderThumbHeight;

    RefPtr<Image> m_fullscreenButton;
    RefPtr<Image> m_muteButton;
    RefPtr<Image> m_unmuteButton;
    RefPtr<Image> m_playButton;
    RefPtr<Image> m_pauseButton;
    RefPtr<Image> m_seekBackButton;
    RefPtr<Image> m_seekForwardButton;
    Page* m_page;
    GRefPtr<GHashTable> m_partsTable;

};

}

#endif // RenderThemeGtk_h
