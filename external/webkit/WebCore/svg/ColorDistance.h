

#ifndef ColorDistance_h
#define ColorDistance_h
#if ENABLE(SVG)

namespace WebCore {
    
    class Color;

    class ColorDistance {
    public:
        ColorDistance();
        ColorDistance(const Color& fromColor, const Color& toColor);
        ColorDistance(int redDiff, int blueDiff, int greenDiff);
        
        ColorDistance scaledDistance(float scaleFactor) const;
        Color addToColorAndClamp(const Color&) const;
        
        static Color addColorsAndClamp(const Color&, const Color&);
        
        bool isZero() const;
        
        float distance() const;
        
    private:
        short m_redDiff;
        short m_greenDiff;
        short m_blueDiff;
    };
}

#endif // ENABLE(SVG)
#endif // ColorDistance_h
