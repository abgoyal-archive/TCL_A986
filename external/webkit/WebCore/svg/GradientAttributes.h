

#ifndef GradientAttributes_h
#define GradientAttributes_h

#if ENABLE(SVG)

namespace WebCore {
    struct GradientAttributes {
        GradientAttributes()
            : m_spreadMethod(SpreadMethodPad)
            , m_boundingBoxMode(true)
            , m_spreadMethodSet(false)
            , m_boundingBoxModeSet(false)
            , m_gradientTransformSet(false)
            , m_stopsSet(false)
        {
        }

        GradientSpreadMethod spreadMethod() const { return m_spreadMethod; }
        bool boundingBoxMode() const { return m_boundingBoxMode; }
        AffineTransform gradientTransform() const { return m_gradientTransform; }
        const Vector<SVGGradientStop>& stops() const { return m_stops; }

        void setSpreadMethod(GradientSpreadMethod value) { m_spreadMethod = value; m_spreadMethodSet = true; }
        void setBoundingBoxMode(bool value) { m_boundingBoxMode = value; m_boundingBoxModeSet = true; }
        void setGradientTransform(const AffineTransform& value) { m_gradientTransform = value; m_gradientTransformSet = true; }
        void setStops(const Vector<SVGGradientStop>& value) { m_stops = value; m_stopsSet = true; } 

        bool hasSpreadMethod() const { return m_spreadMethodSet; }
        bool hasBoundingBoxMode() const { return m_boundingBoxModeSet; }
        bool hasGradientTransform() const { return m_gradientTransformSet; }
        bool hasStops() const { return m_stopsSet; }

    private:
        // Properties
        GradientSpreadMethod m_spreadMethod;
        bool m_boundingBoxMode;
        AffineTransform m_gradientTransform;
        Vector<SVGGradientStop> m_stops;

        // Property states
        bool m_spreadMethodSet : 1;
        bool m_boundingBoxModeSet : 1;
        bool m_gradientTransformSet : 1;
        bool m_stopsSet : 1;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif

// vim:ts=4:noet
