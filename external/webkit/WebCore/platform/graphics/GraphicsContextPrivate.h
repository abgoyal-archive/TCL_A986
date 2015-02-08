

#ifndef GraphicsContextPrivate_h
#define GraphicsContextPrivate_h

#include "Gradient.h"
#include "GraphicsContext.h"
#include "Pattern.h"
#include "TransformationMatrix.h"

namespace WebCore {

    struct GraphicsContextState {
        GraphicsContextState()
            : textDrawingMode(cTextFill)
            , strokeStyle(SolidStroke)
            , strokeThickness(0)
            , strokeColor(Color::black)
            , strokeColorSpace(DeviceColorSpace)
            , fillRule(RULE_NONZERO)
            , fillColor(Color::black)
            , fillColorSpace(DeviceColorSpace)
            , shouldAntialias(true)
            , paintingDisabled(false)
            , shadowBlur(0)
            , shadowsIgnoreTransforms(false)
#if PLATFORM(CAIRO)
            , globalAlpha(1)
#endif
        {
        }

        int textDrawingMode;
        
        StrokeStyle strokeStyle;
        float strokeThickness;
        Color strokeColor;
        ColorSpace strokeColorSpace;
        RefPtr<Gradient> strokeGradient;
        RefPtr<Pattern> strokePattern;
        
        WindRule fillRule;
        Color fillColor;
        ColorSpace fillColorSpace;
        RefPtr<Gradient> fillGradient;
        RefPtr<Pattern> fillPattern;

        bool shouldAntialias;

        bool paintingDisabled;
        
        IntSize shadowSize;
        unsigned shadowBlur;
        Color shadowColor;

        bool shadowsIgnoreTransforms;
#if PLATFORM(CAIRO)
        float globalAlpha;
#elif PLATFORM(QT)
        TransformationMatrix pathTransform;
#endif
    };

    class GraphicsContextPrivate : public Noncopyable {
    public:
        GraphicsContextPrivate()
            : m_updatingControlTints(false)
        {
        }

        GraphicsContextState state;
        Vector<GraphicsContextState> stack;
        bool m_updatingControlTints;
    };

} // namespace WebCore

#endif // GraphicsContextPrivate_h
