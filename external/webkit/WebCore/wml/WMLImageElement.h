

#ifndef WMLImageElement_h
#define WMLImageElement_h

#if ENABLE(WML)
#include "WMLElement.h"
#include "WMLImageLoader.h"

namespace WebCore {

class WMLImageElement : public WMLElement {
public:
    WMLImageElement(const QualifiedName& tagName, Document*);
    virtual ~WMLImageElement();

    virtual bool mapToEntry(const QualifiedName&, MappedAttributeEntry&) const;
    virtual void parseMappedAttribute(MappedAttribute*);

    virtual void attach();
    virtual RenderObject* createRenderer(RenderArena*, RenderStyle*);

    virtual void insertedIntoDocument();
    virtual bool isURLAttribute(Attribute*) const;
    virtual const QualifiedName& imageSourceAttributeName() const;

    String altText() const;

    bool useFallbackAttribute() { return m_useFallbackAttribute; }
    void setUseFallbackAttribute(bool value) { m_useFallbackAttribute = value; }

private:
    WMLImageLoader m_imageLoader;
    bool m_useFallbackAttribute;
};

}

#endif
#endif
