

#include "config.h"
#include "Pattern.h"

#include "AffineTransform.h"
#include "GraphicsContext.h"

#include <ApplicationServices/ApplicationServices.h>

namespace WebCore {

static void patternCallback(void* info, CGContextRef context)
{
    CGImageRef platformImage = static_cast<Image*>(info)->getCGImageRef();
    if (!platformImage)
        return;

    CGRect rect = GraphicsContext(context).roundToDevicePixels(
        FloatRect(0, 0, CGImageGetWidth(platformImage), CGImageGetHeight(platformImage)));
    CGContextDrawImage(context, rect, platformImage);
}

static void patternReleaseCallback(void* info)
{
    static_cast<Image*>(info)->deref();
}

CGPatternRef Pattern::createPlatformPattern(const AffineTransform& userSpaceTransformation) const
{
    IntRect tileRect = tileImage()->rect();

    AffineTransform patternTransform = m_patternSpaceTransformation;
    patternTransform.multiply(userSpaceTransformation);
    patternTransform.scaleNonUniform(1, -1);
    patternTransform.translate(0, -tileRect.height());

    // If FLT_MAX should also be used for xStep or yStep, nothing is rendered. Using fractions of FLT_MAX also
    // result in nothing being rendered.
    // INT_MAX is almost correct, but there seems to be some number wrapping occurring making the fill
    // pattern is not filled correctly.
    // To make error of floating point less than 0.5, we use the half of the number of mantissa of float (1 << 22).
    CGFloat xStep = m_repeatX ? tileRect.width() : (1 << 22);
    CGFloat yStep = m_repeatY ? tileRect.height() : (1 << 22);

    // The pattern will release the tile when it's done rendering in patternReleaseCallback
    tileImage()->ref();

    const CGPatternCallbacks patternCallbacks = { 0, patternCallback, patternReleaseCallback };
    return CGPatternCreate(tileImage(), tileRect, patternTransform, xStep, yStep,
        kCGPatternTilingConstantSpacing, TRUE, &patternCallbacks);
}

}
