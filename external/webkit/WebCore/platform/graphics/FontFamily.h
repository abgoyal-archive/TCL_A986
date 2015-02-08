

#ifndef FontFamily_h
#define FontFamily_h

#include "AtomicString.h"
#include <wtf/RefCounted.h>
#include <wtf/ListRefPtr.h>

namespace WebCore {

class SharedFontFamily;

class FontFamily {
public:
    FontFamily() { }
    FontFamily(const FontFamily&);    
    FontFamily& operator=(const FontFamily&);

    void setFamily(const AtomicString& family) { m_family = family; }
    const AtomicString& family() const { return m_family; }
    bool familyIsEmpty() const { return m_family.isEmpty(); }

    const FontFamily* next() const;

    void appendFamily(PassRefPtr<SharedFontFamily>);
    PassRefPtr<SharedFontFamily> releaseNext();

private:
    AtomicString m_family;
    ListRefPtr<SharedFontFamily> m_next;
};

class SharedFontFamily : public FontFamily, public RefCounted<SharedFontFamily> {
public:
    static PassRefPtr<SharedFontFamily> create()
    {
        return adoptRef(new SharedFontFamily);
    }

private:
    SharedFontFamily() { }
};

bool operator==(const FontFamily&, const FontFamily&);
inline bool operator!=(const FontFamily& a, const FontFamily& b) { return !(a == b); }

inline const FontFamily* FontFamily::next() const
{
    return m_next.get();
}

inline void FontFamily::appendFamily(PassRefPtr<SharedFontFamily> family)
{
    m_next = family;
}

inline PassRefPtr<SharedFontFamily> FontFamily::releaseNext()
{
    return m_next.release();
}

}

#endif
