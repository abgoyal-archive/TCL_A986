

#include "config.h"
#include "TransformationMatrix.h"

#include <windows.h>

namespace WebCore {

TransformationMatrix::operator XFORM() const
{
    XFORM xform;
    xform.eM11 = a();
    xform.eM12 = b();
    xform.eM21 = c();
    xform.eM22 = d();
    xform.eDx = e();
    xform.eDy = f();

    return xform;
}

}
