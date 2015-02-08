
 
#ifndef FontPlatformData_H
#define FontPlatformData_H

#include "FontDescription.h"
#include "AtomicString.h"
#include "CString.h"
#include "StringImpl.h"
#include <wtf/RefPtr.h>

#include <wx/defs.h>
#include <wx/font.h>

namespace WebCore {

class String;

class FontHolder: public WTF::RefCounted<FontHolder>
{
public:
    FontHolder()
        : m_font(0)
    {}

    FontHolder(wxFont* font)
        : m_font(font)
    {}

    wxFont* font() { return m_font; }

private:
    wxFont* m_font;
};

class FontPlatformData {
public:
    enum FontState { UNINITIALIZED, DELETED, VALID };

    FontPlatformData(WTF::HashTableDeletedValueType)
    : m_fontState(DELETED),
      m_font(0)
    { }

    ~FontPlatformData();

    FontPlatformData(const FontDescription&, const AtomicString&);
    FontPlatformData(float size, bool bold, bool italic)
    : m_fontState(UNINITIALIZED)
    , m_font(0)
    {
    }
    
    FontPlatformData() 
    : m_fontState(UNINITIALIZED)
    , m_font(0)
    {
    }
    
    wxFont* font() const {
        return m_font->font();
    }
    
    unsigned hash() const {
        switch (m_fontState) {
        case DELETED:
            return -1;
        case UNINITIALIZED:
            return 0;
        case VALID:
            return computeHash();              
        }
    }

    unsigned computeHash() const;

    bool operator==(const FontPlatformData& other) const
    { 
        if (m_font && m_fontState == VALID && other.m_fontState == VALID && other.m_font) {
            wxFont* thisFont = m_font->font();
            wxFont* otherFont = other.m_font->font();
            return thisFont->IsOk() && otherFont->IsOk() && thisFont->IsSameAs(*otherFont);
        }
        else
            return m_fontState == other.m_fontState;
    }

    bool isHashTableDeletedValue() const { return m_fontState == DELETED; }
    
    bool roundsGlyphAdvances() const { return false; }
    
#if OS(WINDOWS)
    bool useGDI() const;
    HFONT hfont() const;
#endif

#ifndef NDEBUG
    String description() const;
#endif

private:
    WTF::RefPtr<FontHolder> m_font;
    FontState m_fontState;
};

}

#endif
