

#include "config.h"
#include "Pattern.h"

#include "android_graphics.h"
#include "GraphicsContext.h"
#include "SkBitmapRef.h"
#include "SkCanvas.h"
#include "SkColorShader.h"
#include "SkShader.h"
#include "SkPaint.h"

namespace WebCore {

static SkShader::TileMode toTileMode(bool doRepeat) {
    return doRepeat ? SkShader::kRepeat_TileMode : SkShader::kClamp_TileMode;
}

void Pattern::platformDestroy()
{
    m_pattern->safeUnref();
    m_pattern = 0;
}

SkShader* Pattern::platformPattern(const AffineTransform&)
{
    if (m_pattern)
        return m_pattern;

    SkBitmapRef* ref = tileImage()->nativeImageForCurrentFrame();
    if (!ref)
        return 0;
    m_pattern = SkShader::CreateBitmapShader(ref->bitmap(),
                                             toTileMode(m_repeatX),
                                             toTileMode(m_repeatY));
    m_pattern->setLocalMatrix(m_patternSpaceTransformation);
    return m_pattern;
}

void Pattern::setPlatformPatternSpaceTransform()
{
    if (m_pattern)
        m_pattern->setLocalMatrix(m_patternSpaceTransformation);
}

} //namespace
