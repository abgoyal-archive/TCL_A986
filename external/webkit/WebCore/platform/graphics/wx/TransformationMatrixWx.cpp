

#include "config.h"
#include "AffineTransform.h"
#include "TransformationMatrix.h"

#include "Assertions.h"
#include "FloatRect.h"
#include "IntRect.h"

#include <stdio.h>
#include <wx/defs.h>
#include <wx/graphics.h>

namespace WebCore {

#if USE(WXGC)
TransformationMatrix::operator wxGraphicsMatrix() const
{
    wxGraphicsRenderer* renderer = wxGraphicsRenderer::GetDefaultRenderer();
    ASSERT(renderer);
    
    wxGraphicsMatrix matrix = renderer->CreateMatrix(a(), b(), c(), d(), e(), f());
    return matrix;
}

AffineTransform::operator wxGraphicsMatrix() const
{
    wxGraphicsRenderer* renderer = wxGraphicsRenderer::GetDefaultRenderer();
    ASSERT(renderer);
    
    wxGraphicsMatrix matrix = renderer->CreateMatrix(a(), b(), c(), d(), e(), f());
    return matrix;
}
#endif

}
