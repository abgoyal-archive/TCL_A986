

#include <wx/defs.h>
#include <wx/font.h>

void GetTextExtent( const wxFont& font, const wxString& str, wxCoord *width, wxCoord *height,
                            wxCoord *descent = NULL, wxCoord *externalLeading = NULL );

class wxFontProperties{

public:
    wxFontProperties(wxFont* font);
    float GetAscent() { return m_ascent; }
    float GetDescent() { return m_descent; }
    float GetLineGap() { return m_lineGap; }
    float GetLineSpacing() { return m_lineSpacing; }
    float GetXHeight() { return m_xHeight; }

private:
    float m_ascent;
    float m_descent;
    float m_lineGap;
    float m_lineSpacing;
    float m_xHeight;
        
};

bool wxFontContainsCharacters(const wxFont& font, const UChar* characters, int length);
