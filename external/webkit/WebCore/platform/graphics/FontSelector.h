

#ifndef FontSelector_h
#define FontSelector_h

#include <wtf/RefCounted.h>

namespace WebCore {

class AtomicString;
class FontData;
class FontDescription;

class FontSelector : public RefCounted<FontSelector> {
public:
    virtual ~FontSelector() { }
    virtual FontData* getFontData(const FontDescription&, const AtomicString& familyName) = 0;

    virtual void fontCacheInvalidated() { }
};

} // namespace WebCore

#endif // FontSelector_h
