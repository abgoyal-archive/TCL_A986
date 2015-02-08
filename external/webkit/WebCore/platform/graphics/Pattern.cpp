

#include "config.h"
#include "Pattern.h"

#include "Image.h"

namespace WebCore {

Pattern::Pattern(Image* image, bool repeatX, bool repeatY)
    : m_tileImage(image)
    , m_repeatX(repeatX)
    , m_repeatY(repeatY)
#if PLATFORM(SKIA)
    , m_pattern(0)
#endif
{
    ASSERT(image);
}

Pattern::~Pattern()
{
    platformDestroy();
}

void Pattern::setPatternSpaceTransform(const AffineTransform& patternSpaceTransformation)
{
    m_patternSpaceTransformation = patternSpaceTransformation;
    setPlatformPatternSpaceTransform();
}

#if !PLATFORM(SKIA)
void Pattern::platformDestroy()
{
}

void Pattern::setPlatformPatternSpaceTransform()
{
}
#endif

}
