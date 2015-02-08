

#ifndef SVGTextContentElement_h
#define SVGTextContentElement_h

#if ENABLE(SVG)
#include "SVGExternalResourcesRequired.h"
#include "SVGLangSpace.h"
#include "SVGStyledElement.h"
#include "SVGTests.h"

namespace WebCore {

    class SVGLength;

    class SVGTextContentElement : public SVGStyledElement,
                                  public SVGTests,
                                  public SVGLangSpace,
                                  public SVGExternalResourcesRequired {
    public:
        enum SVGLengthAdjustType {
            LENGTHADJUST_UNKNOWN            = 0,
            LENGTHADJUST_SPACING            = 1,
            LENGTHADJUST_SPACINGANDGLYPHS   = 2
        };

        SVGTextContentElement(const QualifiedName&, Document*);
        virtual ~SVGTextContentElement();
        
        virtual bool isValid() const { return SVGTests::isValid(); }
        virtual bool isTextContent() const { return true; }

        unsigned getNumberOfChars() const;
        float getComputedTextLength() const;
        float getSubStringLength(unsigned charnum, unsigned nchars, ExceptionCode&) const;
        FloatPoint getStartPositionOfChar(unsigned charnum, ExceptionCode&) const;
        FloatPoint getEndPositionOfChar(unsigned charnum, ExceptionCode&) const;
        FloatRect getExtentOfChar(unsigned charnum, ExceptionCode&) const;
        float getRotationOfChar(unsigned charnum, ExceptionCode&) const;
        int getCharNumAtPosition(const FloatPoint&) const;
        void selectSubString(unsigned charnum, unsigned nchars, ExceptionCode&) const;

        virtual void parseMappedAttribute(MappedAttribute*);
        virtual void synchronizeProperty(const QualifiedName&);

        bool isKnownAttribute(const QualifiedName&);

    private:
        DECLARE_ANIMATED_PROPERTY(SVGTextContentElement, SVGNames::textLengthAttr, SVGLength, TextLength, textLength)
        DECLARE_ANIMATED_PROPERTY(SVGTextContentElement, SVGNames::lengthAdjustAttr, int, LengthAdjust, lengthAdjust)

        // SVGExternalResourcesRequired
        DECLARE_ANIMATED_PROPERTY(SVGTextContentElement, SVGNames::externalResourcesRequiredAttr, bool, ExternalResourcesRequired, externalResourcesRequired) 
    };

} // namespace WebCore

#endif // ENABLE(SVG)
#endif
