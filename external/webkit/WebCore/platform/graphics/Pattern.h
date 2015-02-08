

#ifndef Pattern_h
#define Pattern_h

#include "AffineTransform.h"

#include <wtf/PassRefPtr.h>
#include <wtf/RefCounted.h>
#include <wtf/RefPtr.h>

#if PLATFORM(CG)
typedef struct CGPattern* CGPatternRef;
typedef CGPatternRef PlatformPatternPtr;
#elif PLATFORM(CAIRO)
#include <cairo.h>
typedef cairo_pattern_t* PlatformPatternPtr;
#elif PLATFORM(SKIA)
class SkShader;
typedef SkShader* PlatformPatternPtr;
#elif PLATFORM(QT)
#include <QBrush>
typedef QBrush PlatformPatternPtr;
#elif PLATFORM(WX)
#if USE(WXGC)
class wxGraphicsBrush;
typedef wxGraphicsBrush* PlatformPatternPtr;
#else
class wxBrush;
typedef wxBrush* PlatformPatternPtr;
#endif // USE(WXGC)
#elif PLATFORM(HAIKU)
#include <interface/GraphicsDefs.h>
typedef pattern* PlatformPatternPtr;
#elif OS(WINCE)
typedef void* PlatformPatternPtr;
#endif

namespace WebCore {

class AffineTransform;
class Image;

class Pattern : public RefCounted<Pattern> {
public:
    static PassRefPtr<Pattern> create(Image* tileImage, bool repeatX, bool repeatY)
    {
        return adoptRef(new Pattern(tileImage, repeatX, repeatY));
    }
    virtual ~Pattern();

    Image* tileImage() const { return m_tileImage.get(); }

    void platformDestroy();

    // Pattern space is an abstract space that maps to the default user space by the transformation 'userSpaceTransformation' 
#if PLATFORM(SKIA)
    PlatformPatternPtr platformPattern(const AffineTransform& userSpaceTransformation);
#else
    PlatformPatternPtr createPlatformPattern(const AffineTransform& userSpaceTransformation) const;
#endif
    void setPatternSpaceTransform(const AffineTransform& patternSpaceTransformation);
    void setPlatformPatternSpaceTransform();

private:
    Pattern(Image*, bool repeatX, bool repeatY);

    RefPtr<Image> m_tileImage;
    bool m_repeatX;
    bool m_repeatY;
    AffineTransform m_patternSpaceTransformation;
    PlatformPatternPtr m_pattern;
};

} //namespace

#endif
