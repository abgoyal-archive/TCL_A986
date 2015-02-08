

#ifndef SVGColor_h
#define SVGColor_h
#if ENABLE(SVG)

#include "CSSValue.h"
#include "Color.h"
#include <wtf/PassRefPtr.h>

namespace WebCore {

    class RGBColor;

    class SVGColor : public CSSValue {
    public:
        static PassRefPtr<SVGColor> create(const String& color)
        {
            return adoptRef(new SVGColor(color));
        }
        static PassRefPtr<SVGColor> create(const Color& color)
        {
            return adoptRef(new SVGColor(color));
        }
        static PassRefPtr<SVGColor> createCurrentColor()
        {
            return adoptRef(new SVGColor(SVG_COLORTYPE_CURRENTCOLOR));
        }

        virtual ~SVGColor();

        enum SVGColorType {
            SVG_COLORTYPE_UNKNOWN                   = 0,
            SVG_COLORTYPE_RGBCOLOR                  = 1,
            SVG_COLORTYPE_RGBCOLOR_ICCCOLOR         = 2,
            SVG_COLORTYPE_CURRENTCOLOR              = 3
        };

        // 'SVGColor' functions
        unsigned short colorType() const;

        PassRefPtr<RGBColor> rgbColor() const;

        static Color colorFromRGBColorString(const String&);

        void setRGBColor(const String& rgbColor) { ExceptionCode ignored = 0; setRGBColor(rgbColor, ignored); }
        void setRGBColor(const String& rgbColor, ExceptionCode&);
        void setRGBColorICCColor(const String& rgbColor, const String& iccColor, ExceptionCode&);
        void setColor(unsigned short colorType, const String& rgbColor, const String& iccColor, ExceptionCode&);

        virtual String cssText() const;

        // Helpers
        const Color& color() const;

    protected:
        SVGColor();
        SVGColor(const String& color);
        SVGColor(const Color&);

    private:
        SVGColor(SVGColorType);

        static void create(int); // compile-time guard 

        virtual bool isSVGColor() const { return true; }

        Color m_color;
        unsigned short m_colorType;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGColor_h

// vim:ts=4:noet
