
 
#include <config.h>

#if ENABLE(SVG)
#include "AffineTransform.h"

#include "SVGException.h"
#include "V8Binding.h"
#include "V8Proxy.h"
#include "V8SVGMatrix.h"
#include "V8SVGPODTypeWrapper.h"

namespace WebCore {

v8::Handle<v8::Value> V8SVGMatrix::multiplyCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SVGMatrix.multiply()");
    if (args.Length() < 1)
        return throwError("Not enough arguments");

    if (!V8SVGMatrix::HasInstance(args[0]))
        return throwError("secondMatrix argument was not a SVGMatrix");

    AffineTransform m1 = *V8SVGPODTypeWrapper<AffineTransform>::toNative(args.Holder());
    AffineTransform m2 = *V8SVGPODTypeWrapper<AffineTransform>::toNative(v8::Handle<v8::Object>::Cast(args[0]));

    RefPtr<V8SVGStaticPODTypeWrapper<AffineTransform> > wrapper = V8SVGStaticPODTypeWrapper<AffineTransform>::create(m1.multLeft(m2));
    return toV8(wrapper.get());
}

v8::Handle<v8::Value> V8SVGMatrix::inverseCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SVGMatrix.inverse()");
    AffineTransform matrix = *V8SVGPODTypeWrapper<AffineTransform>::toNative(args.Holder());
    ExceptionCode ec = 0;
    AffineTransform result = matrix.inverse();

    if (!matrix.isInvertible())
        ec = SVGException::SVG_MATRIX_NOT_INVERTABLE;

    if (ec != 0) {
        V8Proxy::setDOMException(ec);
        return v8::Handle<v8::Value>();
    }

    RefPtr<V8SVGStaticPODTypeWrapper<AffineTransform> > wrapper = V8SVGStaticPODTypeWrapper<AffineTransform>::create(result);
    return toV8(wrapper.get());
}

v8::Handle<v8::Value> V8SVGMatrix::rotateFromVectorCallback(const v8::Arguments& args)
{
    INC_STATS("DOM.SVGMatrix.rotateFromVector()");
    AffineTransform matrix = *V8SVGPODTypeWrapper<AffineTransform>::toNative(args.Holder());
    ExceptionCode ec = 0;
    float x = toFloat(args[0]);
    float y = toFloat(args[1]);
    AffineTransform result = matrix;
    result.rotateFromVector(x, y);
    if (x == 0.0 || y == 0.0)
        ec = SVGException::SVG_INVALID_VALUE_ERR;

    if (ec != 0) {
        V8Proxy::setDOMException(ec);
        return v8::Handle<v8::Value>();
    }

    RefPtr<V8SVGStaticPODTypeWrapper<AffineTransform> > wrapper = V8SVGStaticPODTypeWrapper<AffineTransform>::create(result);
    return toV8(wrapper.get());
}

} // namespace WebCore

#endif
