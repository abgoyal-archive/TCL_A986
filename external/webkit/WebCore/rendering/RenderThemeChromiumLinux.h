

#ifndef RenderThemeChromiumLinux_h
#define RenderThemeChromiumLinux_h

#include "RenderThemeChromiumSkia.h"

namespace WebCore {

    class RenderThemeChromiumLinux : public RenderThemeChromiumSkia {
    public:
        static PassRefPtr<RenderTheme> create();
        virtual String extraDefaultStyleSheet();

        virtual Color systemColor(int cssValidId) const;

        // A method asking if the control changes its tint when the window has focus or not.
        virtual bool controlSupportsTints(const RenderObject*) const;

        // List Box selection color
        virtual Color activeListBoxSelectionBackgroundColor() const;
        virtual Color activeListBoxSelectionForegroundColor() const;
        virtual Color inactiveListBoxSelectionBackgroundColor() const;
        virtual Color inactiveListBoxSelectionForegroundColor() const;

        virtual Color platformActiveSelectionBackgroundColor() const;
        virtual Color platformInactiveSelectionBackgroundColor() const;
        virtual Color platformActiveSelectionForegroundColor() const;
        virtual Color platformInactiveSelectionForegroundColor() const;

        virtual void adjustSliderThumbSize(RenderObject*) const;

        static void setCaretBlinkInterval(double interval);
        virtual double caretBlinkIntervalInternal() const;

        static void setSelectionColors(unsigned activeBackgroundColor,
                                       unsigned activeForegroundColor,
                                       unsigned inactiveBackgroundColor,
                                       unsigned inactiveForegroundColor);

        static void setScrollbarColors(unsigned inactive_color,
                                       unsigned active_color,
                                       unsigned track_color);
        static unsigned thumbInactiveColor() { return m_thumbInactiveColor; }
        static unsigned thumbActiveColor() { return m_thumbActiveColor; }
        static unsigned trackColor() { return m_trackColor; }

    private:
        RenderThemeChromiumLinux();
        virtual ~RenderThemeChromiumLinux();

        // A general method asking if any control tinting is supported at all.
        virtual bool supportsControlTints() const;

        static double m_caretBlinkInterval;

        static unsigned m_activeSelectionBackgroundColor;
        static unsigned m_activeSelectionForegroundColor;
        static unsigned m_inactiveSelectionBackgroundColor;
        static unsigned m_inactiveSelectionForegroundColor;

        static unsigned m_thumbInactiveColor;
        static unsigned m_thumbActiveColor;
        static unsigned m_trackColor;
    };

} // namespace WebCore

#endif // RenderThemeChromiumLinux_h
