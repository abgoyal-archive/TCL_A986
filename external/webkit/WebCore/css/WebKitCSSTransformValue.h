

#ifndef WebKitCSSTransformValue_h
#define WebKitCSSTransformValue_h

#include "CSSValueList.h"
#include <wtf/PassRefPtr.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class WebKitCSSTransformValue : public CSSValueList {
public:
    // NOTE: these have to match the values in the IDL
    enum TransformOperationType {
        UnknownTransformOperation,
        TranslateTransformOperation,
        TranslateXTransformOperation,
        TranslateYTransformOperation,
        RotateTransformOperation,
        ScaleTransformOperation,
        ScaleXTransformOperation,
        ScaleYTransformOperation,
        SkewTransformOperation,
        SkewXTransformOperation,
        SkewYTransformOperation,
        MatrixTransformOperation,
        TranslateZTransformOperation,
        Translate3DTransformOperation,
        RotateXTransformOperation,
        RotateYTransformOperation,
        RotateZTransformOperation,
        Rotate3DTransformOperation,
        ScaleZTransformOperation,
        Scale3DTransformOperation,
        PerspectiveTransformOperation,
        Matrix3DTransformOperation
    };

    static PassRefPtr<WebKitCSSTransformValue> create(TransformOperationType type)
    {
        return adoptRef(new WebKitCSSTransformValue(type));
    }

    virtual ~WebKitCSSTransformValue();

    virtual String cssText() const;
 
    TransformOperationType operationType() const { return m_type; }
    
private:
    WebKitCSSTransformValue(TransformOperationType);

    virtual bool isWebKitCSSTransformValue() const { return true; }

    TransformOperationType m_type;
};

}

#endif
