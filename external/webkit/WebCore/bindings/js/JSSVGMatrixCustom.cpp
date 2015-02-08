

#include "config.h"

#if ENABLE(SVG)
#include "JSSVGMatrix.h"

#include "AffineTransform.h"
#include "SVGException.h"
#include <runtime/Error.h>

using namespace JSC;

namespace WebCore {

JSValue JSSVGMatrix::multiply(ExecState* exec, const ArgList& args)
{
    if (args.size() < 1)
        return throwError(exec, SyntaxError, "Not enough arguments");

    if (!args.at(0).inherits(&JSSVGMatrix::s_info))
        return throwError(exec, TypeError, "secondMatrix argument was not a SVGMatrix");

    JSSVGMatrix* matrixObj = static_cast<JSSVGMatrix*>(asObject(args.at(0)));

    AffineTransform m1(*impl());
    AffineTransform m2(*(matrixObj->impl()));

    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);
    return toJS(exec, globalObject(), JSSVGStaticPODTypeWrapper<AffineTransform>::create(m1.multLeft(m2)).get(), context);
}

JSValue JSSVGMatrix::inverse(ExecState* exec, const ArgList&)
{
    AffineTransform imp(*impl());

    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);
    JSValue result = toJS(exec, globalObject(), JSSVGStaticPODTypeWrapper<AffineTransform>::create(imp.inverse()).get(), context);

    if (!imp.isInvertible())
        setDOMException(exec, SVGException::SVG_MATRIX_NOT_INVERTABLE);

    return result;
}

JSValue JSSVGMatrix::rotateFromVector(ExecState* exec, const ArgList& args)
{
    AffineTransform imp(*impl());
 
    float x = args.at(0).toFloat(exec);
    float y = args.at(1).toFloat(exec);

    SVGElement* context = JSSVGContextCache::svgContextForDOMObject(this);
    JSValue result = toJS(exec, globalObject(), JSSVGStaticPODTypeWrapper<AffineTransform>::create(imp.rotateFromVector(x, y)).get(), context);

    if (x == 0.0 || y == 0.0)
        setDOMException(exec, SVGException::SVG_INVALID_VALUE_ERR);

    return result;
}

}

#endif // ENABLE(SVG)
