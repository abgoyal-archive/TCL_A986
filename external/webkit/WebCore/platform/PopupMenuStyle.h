

#ifndef PopupMenuStyle_h
#define PopupMenuStyle_h

#include "Color.h"
#include "Font.h"
#include "Length.h"
#include "TextDirection.h"

namespace WebCore {

class PopupMenuStyle {
public:
    PopupMenuStyle(const Color& foreground, const Color& background, const Font& font, bool visible, Length textIndent, TextDirection textDirection)
        : m_foregroundColor(foreground)
        , m_backgroundColor(background)
        , m_font(font)
        , m_visible(visible)
        , m_textIndent(textIndent)
        , m_textDirection(textDirection)
    {
    }
    
    const Color& foregroundColor() const { return m_foregroundColor; }
    const Color& backgroundColor() const { return m_backgroundColor; }
    const Font& font() const { return m_font; }
    bool isVisible() const { return m_visible; }
    Length textIndent() const { return m_textIndent; }
    TextDirection textDirection() const { return m_textDirection; }

private:
    Color m_foregroundColor;
    Color m_backgroundColor;
    Font m_font;
    bool m_visible;
    Length m_textIndent;
    TextDirection m_textDirection;
};

} // namespace WebCore

#endif // PopupMenuStyle_h
