

#include "config.h"
#include "Pattern.h"

#include "AffineTransform.h"
#include "GraphicsContext.h"

#include <cairo.h>

namespace WebCore {

cairo_pattern_t* Pattern::createPlatformPattern(const AffineTransform&) const
{
    cairo_surface_t* surface = tileImage()->nativeImageForCurrentFrame();
    if (!surface)
        return 0;

    cairo_pattern_t* pattern = cairo_pattern_create_for_surface(surface);

    // cairo merges patter space and user space itself
    cairo_matrix_t matrix = m_patternSpaceTransformation;
    cairo_matrix_invert(&matrix);
    cairo_pattern_set_matrix(pattern, &matrix);

    if (m_repeatX || m_repeatY)
        cairo_pattern_set_extend(pattern, CAIRO_EXTEND_REPEAT);
    return pattern;
}

}
