

#include "config.h"
#include "AffineTransform.h"
#include "TransformationMatrix.h"

#if PLATFORM(CG)

#include <CoreGraphics/CGAffineTransform.h>
#include "FloatConversion.h"

namespace WebCore {

TransformationMatrix::operator CGAffineTransform() const
{
    return CGAffineTransformMake(narrowPrecisionToCGFloat(a()),
                                 narrowPrecisionToCGFloat(b()),
                                 narrowPrecisionToCGFloat(c()),
                                 narrowPrecisionToCGFloat(d()),
                                 narrowPrecisionToCGFloat(e()),
                                 narrowPrecisionToCGFloat(f()));
}

AffineTransform::operator CGAffineTransform() const
{
    return CGAffineTransformMake(narrowPrecisionToCGFloat(a()),
                                 narrowPrecisionToCGFloat(b()),
                                 narrowPrecisionToCGFloat(c()),
                                 narrowPrecisionToCGFloat(d()),
                                 narrowPrecisionToCGFloat(e()),
                                 narrowPrecisionToCGFloat(f()));
}

}

#endif // PLATFORM(CG)
