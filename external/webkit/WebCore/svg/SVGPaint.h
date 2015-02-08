

#ifndef SVGPaint_h
#define SVGPaint_h
#if ENABLE(SVG)

#include "SVGColor.h"
#include "PlatformString.h"

namespace WebCore {

    class SVGPaint : public SVGColor {
    public:
        enum SVGPaintType {
            SVG_PAINTTYPE_UNKNOWN               = 0,
            SVG_PAINTTYPE_RGBCOLOR              = 1,
            SVG_PAINTTYPE_RGBCOLOR_ICCCOLOR     = 2,
            SVG_PAINTTYPE_NONE                  = 101,
            SVG_PAINTTYPE_CURRENTCOLOR          = 102,
            SVG_PAINTTYPE_URI_NONE              = 103,
            SVG_PAINTTYPE_URI_CURRENTCOLOR      = 104,
            SVG_PAINTTYPE_URI_RGBCOLOR          = 105,
            SVG_PAINTTYPE_URI_RGBCOLOR_ICCCOLOR = 106,
            SVG_PAINTTYPE_URI                   = 107
        };

        static PassRefPtr<SVGPaint> create()
        {
            return adoptRef(new SVGPaint);
        }
        static PassRefPtr<SVGPaint> create(SVGPaintType type)
        {
            return adoptRef(new SVGPaint(type));
        }
        static PassRefPtr<SVGPaint> create(const Color& color)
        {
            return adoptRef(new SVGPaint(color));
        }
        static PassRefPtr<SVGPaint> create(SVGPaintType type, const String& uri)
        {
            return adoptRef(new SVGPaint(type, uri, String(), String()));
        }
        static PassRefPtr<SVGPaint> create(const String& uri, const Color& color)
        {
            return adoptRef(new SVGPaint(uri, color));
        }

        virtual ~SVGPaint();

        // 'SVGPaint' functions
        SVGPaintType paintType() const { return m_paintType; }
        String uri() const;

        void setUri(const String&);
        void setPaint(SVGPaintType, const String& uri, const String& rgbPaint, const String& iccPaint, ExceptionCode&);

        virtual String cssText() const;
        
        static SVGPaint* defaultFill();
        static SVGPaint* defaultStroke();

    private:
        SVGPaint();
        SVGPaint(const String& uri);
        SVGPaint(SVGPaintType);
        SVGPaint(SVGPaintType, const String& uri, const String& rgbPaint, const String& iccPaint);
        SVGPaint(const Color& c);
        SVGPaint(const String& uri, const Color& c);

        virtual bool isSVGPaint() const { return true; }

        SVGPaintType m_paintType;
        String m_uri;
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif // SVGPaint_h

// vim:ts=4:noet
