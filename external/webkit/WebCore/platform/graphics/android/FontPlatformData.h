

// This file is part of the internal font implementation.  It should not be included by anyone other than
// FontMac.cpp, FontWin.cpp and Font.cpp.

#ifndef FontPlatformData_H
#define FontPlatformData_H

#ifndef NDEBUG
#include "PlatformString.h"
#endif

#include "StringImpl.h"

class SkPaint;
class SkTypeface;

namespace WebCore {

class FontPlatformData {
public:
    static FontPlatformData Deleted() {
        return FontPlatformData(NULL, -1, false, false);
    }

    FontPlatformData();
    FontPlatformData(const FontPlatformData&);
    FontPlatformData(SkTypeface*, float textSize, bool fakeBold, bool fakeItalic);
    FontPlatformData(const FontPlatformData& src, float textSize);
    FontPlatformData(float size, bool syntheticBold, bool syntheticOblique);

    ~FontPlatformData();

    FontPlatformData(WTF::HashTableDeletedValueType)
        : mTypeface(hashTableDeletedFontValue()) { }
    bool isHashTableDeletedValue() const {
        return mTypeface == hashTableDeletedFontValue();
    }

    FontPlatformData& operator=(const FontPlatformData&);
    bool operator==(const FontPlatformData& a) const;

    void     setupPaint(SkPaint*) const;
    float size() const { return mTextSize; }
    unsigned hash() const;

#ifndef NDEBUG
    String description() const { return ""; }
#endif

private:
    SkTypeface* mTypeface;
    float       mTextSize;
    bool        mFakeBold;
    bool        mFakeItalic;

    static SkTypeface* hashTableDeletedFontValue() {
        return reinterpret_cast<SkTypeface*>(-1);
    }
};
    
} /* namespace */

#endif
