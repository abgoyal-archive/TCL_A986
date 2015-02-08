

#include "config.h"
#include "AffineTransform.h"
#include "TransformationMatrix.h"

#include "IntRect.h"
#include "FloatRect.h"

namespace WebCore {

TransformationMatrix::operator QTransform() const
{
    return QTransform(m11(), m12(), m14(), m21(), m22(), m24(), m41(), m42(), m44());
}

AffineTransform::operator QTransform() const
{
    return QTransform(a(), b(), c(), d(), e(), f());
}

}

// vim: ts=4 sw=4 et
