

#ifndef CSSCanvasValue_h
#define CSSCanvasValue_h

#include "CSSImageGeneratorValue.h"
#include "HTMLCanvasElement.h"

namespace WebCore {

class Document;

class CSSCanvasValue : public CSSImageGeneratorValue, private CanvasObserver {
public:
    static PassRefPtr<CSSCanvasValue> create() { return adoptRef(new CSSCanvasValue); }
    virtual ~CSSCanvasValue();

    virtual String cssText() const;

    virtual Image* image(RenderObject*, const IntSize&);
    virtual bool isFixedSize() const { return true; }
    virtual IntSize fixedSize(const RenderObject*);

    void setName(const String& name) { m_name = name; }

private:
    CSSCanvasValue()
        : m_element(0)
    {
    }

    virtual void canvasChanged(HTMLCanvasElement*, const FloatRect& changedRect);
    virtual void canvasResized(HTMLCanvasElement*);
    virtual void canvasDestroyed(HTMLCanvasElement*);

    HTMLCanvasElement* element(Document*);
     
    // The name of the canvas.
    String m_name;
    // The document supplies the element and owns it.
    HTMLCanvasElement* m_element;
};

} // namespace WebCore

#endif // CSSCanvasValue_h
