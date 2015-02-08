

#ifndef FontFamilyValue_h
#define FontFamilyValue_h

#include "CSSPrimitiveValue.h"
#include "PlatformString.h"

namespace WebCore {

class FontFamilyValue : public CSSPrimitiveValue {
public:
    static PassRefPtr<FontFamilyValue> create(const String& familyName)
    {
        return adoptRef(new FontFamilyValue(familyName));
    }

    void appendSpaceSeparated(const UChar* characters, unsigned length);

    const String& familyName() const { return m_familyName; }

    virtual String cssText() const;

private:
    FontFamilyValue(const String& familyName);

    String m_familyName;
};

} // namespace

#endif
