

#include "config.h"
#include "WebKitCSSTransformValue.h"

#include "CSSValueList.h"
#include "PlatformString.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

WebKitCSSTransformValue::WebKitCSSTransformValue(TransformOperationType op)
    : CSSValueList(false)
    , m_type(op)
{
}

WebKitCSSTransformValue::~WebKitCSSTransformValue()
{
}

String WebKitCSSTransformValue::cssText() const
{
    String result;
    switch (m_type) {
        case TranslateTransformOperation:
            result += "translate(";
            break;
        case TranslateXTransformOperation:
            result += "translateX(";
            break;
        case TranslateYTransformOperation:
            result += "translateY(";
            break;
        case RotateTransformOperation:
            result += "rotate(";
            break;
        case ScaleTransformOperation:
            result += "scale(";
            break;
        case ScaleXTransformOperation:
            result += "scaleX(";
            break;
        case ScaleYTransformOperation:
            result += "scaleY(";
            break;
        case SkewTransformOperation:
            result += "skew(";
            break;
        case SkewXTransformOperation:
            result += "skewX(";
            break;
        case SkewYTransformOperation:
            result += "skewY(";
            break;
        case MatrixTransformOperation:
            result += "matrix(";
            break;
        case TranslateZTransformOperation:
            result += "translateZ(";
            break;
        case Translate3DTransformOperation:
            result += "translate3d(";
            break;
        case RotateXTransformOperation:
            result += "rotateX(";
            break;
        case RotateYTransformOperation:
            result += "rotateY(";
            break;
        case RotateZTransformOperation:
            result += "rotateZ(";
            break;
        case Rotate3DTransformOperation:
            result += "rotate3d(";
            break;
        case ScaleZTransformOperation:
            result += "scaleZ(";
            break;
        case Scale3DTransformOperation:
            result += "scale3d(";
            break;
        case PerspectiveTransformOperation:
            result += "perspective(";
            break;
        case Matrix3DTransformOperation:
            result += "matrix3d(";
            break;
        default:
            break;
    }
    
    result += CSSValueList::cssText();
    
    result += ")";
    return result;
}

}
