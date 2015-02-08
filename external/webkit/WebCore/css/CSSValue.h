

#ifndef CSSValue_h
#define CSSValue_h

#include "StyleBase.h"

#include "CSSParserValues.h"
#include "KURLHash.h"
#include <wtf/ListHashSet.h>
#include <wtf/RefPtr.h>

namespace WebCore {

class CSSStyleSheet;

typedef int ExceptionCode;

class CSSValue : public RefCounted<CSSValue> {
public:
    // FIXME: Change name to Type.
    enum UnitTypes {
        CSS_INHERIT = 0,
        CSS_PRIMITIVE_VALUE = 1,
        CSS_VALUE_LIST = 2,
        CSS_CUSTOM = 3,
        CSS_INITIAL = 4
    };

    virtual ~CSSValue() { }

    // FIXME: Change this to return UnitTypes.
    virtual unsigned short cssValueType() const { return CSS_CUSTOM; }

    virtual String cssText() const = 0;
    void setCssText(const String&, ExceptionCode&) { } // FIXME: Not implemented.

    virtual bool isFontValue() const { return false; }
    virtual bool isImageGeneratorValue() const { return false; }
    virtual bool isImageValue() const { return false; }
    virtual bool isImplicitInitialValue() const { return false; }
    virtual bool isPrimitiveValue() const { return false; }
    virtual bool isTimingFunctionValue() const { return false; }
    virtual bool isValueList() const { return false; }
    virtual bool isWebKitCSSTransformValue() const { return false; }

#if ENABLE(SVG)
    virtual bool isSVGColor() const { return false; }
    virtual bool isSVGPaint() const { return false; }
#endif

    virtual bool isVariableDependentValue() const { return false; }
    virtual CSSParserValue parserValue() const { ASSERT_NOT_REACHED(); return CSSParserValue(); }

    virtual void addSubresourceStyleURLs(ListHashSet<KURL>&, const CSSStyleSheet*) { }
};

} // namespace WebCore

#endif // CSSValue_h
