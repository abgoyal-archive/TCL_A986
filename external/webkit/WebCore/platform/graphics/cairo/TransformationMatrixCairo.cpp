

#include "config.h"
#include "AffineTransform.h"
#include "TransformationMatrix.h"

#include "IntRect.h"
#include "FloatRect.h"

#include <cairo.h>

namespace WebCore {

TransformationMatrix::operator cairo_matrix_t() const
{
    cairo_matrix_t m;

    cairo_matrix_init (&m,
                       a(),
                       b(),
                       c(),
                       d(),
                       e(),
                       f());
    return m;
}

AffineTransform::operator cairo_matrix_t() const
{
    cairo_matrix_t m;

    cairo_matrix_init (&m,
                       a(),
                       b(),
                       c(),
                       d(),
                       e(),
                       f());
    return m;
}

}

// vim: ts=4 sw=4 et
