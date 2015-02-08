

#ifndef Pen_h
#define Pen_h

#include "Color.h"

#if PLATFORM(WX)
class wxPen;
#endif

namespace WebCore {

class Pen {
public:
    enum PenStyle {
        NoPen,
        SolidLine,
        DotLine,
        DashLine
    };

    Pen(const Color &c = Color::black, unsigned w = 0, PenStyle ps = SolidLine);

    const Color &color() const;
    unsigned width() const;
    PenStyle style() const;

    void setColor(const Color &);
    void setWidth(unsigned);
    void setStyle(PenStyle);

    bool operator==(const Pen &) const;
    bool operator!=(const Pen &) const;
    
#if PLATFORM(WX)
    Pen(const wxPen&);
    operator wxPen() const;
#endif

private:
    PenStyle  m_style;
    unsigned  m_width;
    Color     m_color;
};

}

#endif
