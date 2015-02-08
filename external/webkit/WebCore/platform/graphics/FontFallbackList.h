

// This file has no guards on purpose in order to detect redundant includes. This is a private header
// and so this may catch someone trying to include this file in public cpp files.

#include "FontSelector.h"
#include "SimpleFontData.h"
#include <wtf/Forward.h>

namespace WebCore {

class Font;
class GlyphPageTreeNode;
class GraphicsContext;
class IntRect;
class FontDescription;
class FontPlatformData;
class FontSelector;

const int cAllFamiliesScanned = -1;

class FontFallbackList : public RefCounted<FontFallbackList> {
public:
    static PassRefPtr<FontFallbackList> create() { return adoptRef(new FontFallbackList()); }

    ~FontFallbackList() { releaseFontData(); }
    void invalidate(PassRefPtr<FontSelector>);
    
    bool isFixedPitch(const Font* f) const { if (m_pitch == UnknownPitch) determinePitch(f); return m_pitch == FixedPitch; };
    void determinePitch(const Font*) const;

    bool loadingCustomFonts() const { return m_loadingCustomFonts; }

    FontSelector* fontSelector() const { return m_fontSelector.get(); }
    unsigned generation() const { return m_generation; }

private:
    FontFallbackList();

    const SimpleFontData* primarySimpleFontData(const Font* f)
    { 
        ASSERT(isMainThread());
        if (!m_cachedPrimarySimpleFontData)
            m_cachedPrimarySimpleFontData = primaryFontData(f)->fontDataForCharacter(' ');
        return m_cachedPrimarySimpleFontData;
    }

    const FontData* primaryFontData(const Font* f) const { return fontDataAt(f, 0); }
    const FontData* fontDataAt(const Font*, unsigned index) const;
    const FontData* fontDataForCharacters(const Font*, const UChar*, int length) const;
    
    void setPlatformFont(const FontPlatformData&);

    void releaseFontData();

    mutable Vector<pair<const FontData*, bool>, 1> m_fontList;
    mutable HashMap<int, GlyphPageTreeNode*> m_pages;
    mutable GlyphPageTreeNode* m_pageZero;
    mutable const SimpleFontData* m_cachedPrimarySimpleFontData;
    RefPtr<FontSelector> m_fontSelector;
    mutable int m_familyIndex;
    mutable Pitch m_pitch;
    mutable bool m_loadingCustomFonts;
    unsigned m_generation;

    friend class Font;
};

}

