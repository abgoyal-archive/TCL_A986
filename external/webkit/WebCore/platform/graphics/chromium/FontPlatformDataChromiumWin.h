

#ifndef FontPlatformDataWin_h
#define FontPlatformDataWin_h

#include "config.h"

#include "StringImpl.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>

#include <usp10.h>

typedef struct HFONT__ *HFONT;

namespace WebCore {

class FontDescription;
class String;

class FontPlatformData {
public:
    // Used for deleted values in the font cache's hash tables. The hash table
    // will create us with this structure, and it will compare other values
    // to this "Deleted" one. It expects the Deleted one to be differentiable
    // from the NULL one (created with the empty constructor), so we can't just
    // set everything to NULL.
    FontPlatformData(WTF::HashTableDeletedValueType);
    FontPlatformData();
    FontPlatformData(HFONT, float size);
    FontPlatformData(float size, bool bold, bool oblique);
    FontPlatformData(const FontPlatformData&);

    FontPlatformData& operator=(const FontPlatformData&);

    bool isHashTableDeletedValue() const { return m_font == hashTableDeletedFontValue(); }

    ~FontPlatformData();

    HFONT hfont() const { return m_font ? m_font->hfont() : 0; }
    float size() const { return m_size; }

    unsigned hash() const
    { 
        return m_font ? m_font->hash() : NULL;
    }

    bool operator==(const FontPlatformData& other) const
    { 
        return m_font == other.m_font && m_size == other.m_size;
    }

#ifndef NDEBUG
    String description() const;
#endif

    SCRIPT_FONTPROPERTIES* scriptFontProperties() const;
    SCRIPT_CACHE* scriptCache() const { return &m_scriptCache; }

private:
    // We refcount the internal HFONT so that FontPlatformData can be
    // efficiently copied. WebKit depends on being able to copy it, and we
    // don't really want to re-create the HFONT.
    class RefCountedHFONT : public RefCounted<RefCountedHFONT> {
    public:
        static PassRefPtr<RefCountedHFONT> create(HFONT hfont)
        {
            return adoptRef(new RefCountedHFONT(hfont));
        }

        ~RefCountedHFONT();

        HFONT hfont() const { return m_hfont; }
        unsigned hash() const
        {
            return StringImpl::computeHash(reinterpret_cast<const UChar*>(&m_hfont), sizeof(HFONT) / sizeof(UChar));
        }

        bool operator==(const RefCountedHFONT& other) const
        {
            return m_hfont == other.m_hfont;
        }

    private:
        // The create() function assumes there is already a refcount of one 
        // so it can do adoptRef.
        RefCountedHFONT(HFONT hfont) : m_hfont(hfont)
        {
        }

        HFONT m_hfont;
    };

    static RefCountedHFONT* hashTableDeletedFontValue();

    RefPtr<RefCountedHFONT> m_font;
    float m_size;  // Point size of the font in pixels.

    mutable SCRIPT_CACHE m_scriptCache;
    mutable SCRIPT_FONTPROPERTIES* m_scriptFontProperties;
};

} // WebCore

#endif // FontPlatformDataWin_h
