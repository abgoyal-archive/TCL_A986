

#ifndef HTMLAreaElement_h
#define HTMLAreaElement_h

#include "HTMLAnchorElement.h"
#include "IntSize.h"
#include <wtf/OwnArrayPtr.h>

namespace WebCore {

class HitTestResult;
class HTMLImageElement;
class Path;

class HTMLAreaElement : public HTMLAnchorElement {
public:
    static PassRefPtr<HTMLAreaElement> create(const QualifiedName&, Document*);

    bool isDefault() const { return m_shape == Default; }

    bool mapMouseEvent(int x, int y, const IntSize&, HitTestResult&);

    IntRect getRect(RenderObject*) const;
    Path getPath(RenderObject*) const;
    
    // Convenience method to get the parent map's image.
    HTMLImageElement* imageElement() const;
    
    KURL href() const;

    bool noHref() const;
    void setNoHref(bool);

private:
    HTMLAreaElement(const QualifiedName&, Document*);

    virtual HTMLTagStatus endTagRequirement() const { return TagStatusForbidden; }
    virtual int tagPriority() const { return 0; }
    virtual void parseMappedAttribute(MappedAttribute*);
    virtual bool supportsFocus() const;
    virtual String target() const;
    virtual bool isKeyboardFocusable(KeyboardEvent*) const;
    virtual bool isFocusable() const;
    virtual void updateFocusAppearance(bool /*restorePreviousSelection*/);
    virtual void dispatchBlurEvent();
    
    enum Shape { Default, Poly, Rect, Circle, Unknown };
    Path getRegion(const IntSize&) const;

    OwnPtr<Path> m_region;
    OwnArrayPtr<Length> m_coords;
    int m_coordsLen;
    IntSize m_lastSize;
    Shape m_shape;
};

} //namespace

#endif
